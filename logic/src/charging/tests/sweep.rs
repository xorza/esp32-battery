//! Randomized sweep: invariants that must hold for every input sequence,
//! not just the ones someone thought to write down.

use super::*;

/// Deterministic xorshift64. Enough to shuffle the supervisor through state
/// combinations no hand-written scenario reaches, while staying exactly
/// reproducible from the seed a failure prints.
struct Rng(u64);

impl Rng {
    fn new(seed: u64) -> Self {
        // xorshift64 is a fixed point at zero.
        Self(seed | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 << 13;
        self.0 ^= self.0 >> 7;
        self.0 ^= self.0 << 17;
        self.0
    }

    /// Uniform in `0..n`.
    fn below(&mut self, n: u64) -> u64 {
        self.next() % n
    }
}

/// A poll drawn from the plausible space: mostly healthy, occasionally a
/// failed read, a drifted setpoint, a missing or NaN sample, or a buck
/// that disagrees with the supervisor about its own output.
fn random_poll(rng: &mut Rng, s: &ChargeSupervisor) -> PollResult {
    let profile = lfp_4s();
    let setpoints = match rng.below(1000) {
        0..=14 => None,
        15..=16 => Some(Setpoints {
            v_set: 12.0,
            i_set: profile.regulation_a,
        }),
        _ => Some(s.expected_setpoints()),
    };
    let battery = match rng.below(1000) {
        0..=19 => None,
        20..=24 => Some(BatterySample {
            voltage: f32::NAN,
            current: -1.0,
        }),
        // 12.00–15.99 V spans the CC ramp, the CV plateau (14.4) and the
        // OV trip (14.6).
        //
        // Current is bimodal on purpose. Drawn uniformly from 0–12 A it
        // sits above the 2.5 A tail on most ticks, so the *leaky* exit
        // window never nets its 60 s and Absorb→Float — the step-down, the
        // transition that must cycle the output — fires zero times in
        // 100k ticks. So 85% of samples come from a band straddling the
        // tail instead: both sides of the gate still get drawn, but below
        // it often enough that the window can actually net out mid-run.
        _ => Some(BatterySample {
            voltage: 12.0 + (rng.below(400) as f32) / 100.0,
            current: -if rng.below(100) < 85 {
                (rng.below(260) as f32) / 100.0
            } else {
                (rng.below(1200) as f32) / 100.0
            },
        }),
    };
    let output = match rng.below(1000) {
        0 => None,
        1..=3 => Some(BuckOutput::Off {
            cause: match rng.below(4) {
                0 => ProtectionStatus::Lvp,
                1 => ProtectionStatus::Otp,
                2 => ProtectionStatus::Ovp,
                _ => ProtectionStatus::Normal,
            },
        }),
        _ => Some(expected_output(s)),
    };
    PollResult {
        setpoints,
        output,
        battery,
    }
}

#[test]
fn invariants_hold_under_randomized_input() {
    const SEEDS: u64 = 200;
    const TICKS: u32 = 1200;
    let profile = lfp_4s();
    let ov_trip = profile.absorb_v + OV_MARGIN_V;

    // Meta-coverage: a sweep that never reaches these would pass vacuously.
    let mut saw_enable = 0u32;
    let mut saw_step_up = 0u32;
    let mut saw_step_down = 0u32;
    let mut saw_disable = 0u32;

    for seed in 1..=SEEDS {
        let mut rng = Rng::new(seed);
        let mut s = ChargeSupervisor::new(profile);
        // Boot writes float_v, so that is the committed setpoint until an
        // UpdateVoltage is committed.
        let mut committed_v = profile.float_v;
        let mut latched = false;

        for tick in 0..TICKS {
            let p = random_poll(&mut rng, &s);
            let a = s.tick(p, TICK);

            // The drift check compares against the last *committed* target,
            // which is the invariant SettingsDrift exists to police.
            assert!(
                approx(s.expected_setpoints().v_set, committed_v),
                "seed {seed} tick {tick}: expected V_SET {} != committed {committed_v}",
                s.expected_setpoints().v_set
            );

            // A latched fault is terminal and supersedes any inhibit.
            assert!(
                !(s.fault().is_some() && s.inhibit().is_some()),
                "seed {seed} tick {tick}: fault and inhibit both set"
            );

            // Tripped is absorbing: nothing may re-energise after a latch.
            if latched {
                assert!(
                    matches!(a, Action::None | Action::DisableOutput(_)),
                    "seed {seed} tick {tick}: {a:?} emitted after a latch"
                );
            }

            match a {
                Action::None => {}
                Action::EnableOutput(t) => {
                    saw_enable += 1;
                    // Never energise into an over-volt pack. The sample is
                    // the one the supervisor accepted, so it is finite.
                    let b = p.battery.expect("EnableOutput requires a sample");
                    assert!(
                        b.voltage <= ov_trip,
                        "seed {seed} tick {tick}: EnableOutput at {} V, over the {ov_trip} V trip",
                        b.voltage
                    );
                    if rng.below(100) < 80 {
                        s.commit_enable(t);
                    }
                }
                Action::UpdateVoltage(t) => {
                    // `cycle_output` must be exactly "is this a step down".
                    // Getting it wrong back-feeds the buck's low-side FET.
                    //
                    // Read after the tick on purpose: the phase commit is
                    // deferred to `commit_voltage`, so `expected_setpoints`
                    // still reports the voltage the buck is regulating to
                    // right now — which is what `target_v` is stepping from.
                    let live = s.expected_setpoints().v_set;
                    if t.cycle_output {
                        saw_step_down += 1;
                        assert!(
                            t.target_v < live,
                            "seed {seed} tick {tick}: cycle_output on a step from {live} to {}",
                            t.target_v
                        );
                    } else {
                        saw_step_up += 1;
                        assert!(
                            t.target_v > live,
                            "seed {seed} tick {tick}: no cycle_output on a step from {live} to {}",
                            t.target_v
                        );
                    }
                    if rng.below(100) < 80 {
                        committed_v = t.target_v;
                        s.commit_voltage(t);
                    }
                }
                Action::DisableOutput(t) => {
                    saw_disable += 1;
                    latched = true;
                    if rng.below(100) < 80 {
                        s.commit_disable(t);
                    }
                }
            }
        }
    }

    // Meta-coverage. The RNG is deterministic, so these counts are exact and
    // reproducible; the thresholds sit well under what the sweep currently
    // reaches (246/246/10/252) so ordinary tuning won't trip them, but a
    // change that stops the sweep reaching a path will. Absorb→Float is the
    // rare one — it needs ~90 uninterrupted ticks for the leaky exit window
    // to net out — and carries dedicated coverage in
    // `absorb_to_float_emits_step_down_with_output_cycle` and the apply
    // tests, so a low bar here is enough.
    assert!(
        saw_enable >= 100
            && saw_step_up >= 100
            && saw_step_down >= 5
            && saw_disable >= 100,
        "sweep stopped covering: enable={saw_enable} up={saw_step_up} \
         down={saw_step_down} disable={saw_disable}"
    );
}
