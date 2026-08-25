use super::*;

const CAP: usize = HISTORY_CAPACITY;
const HALF: usize = CAP / 2;

/// Push n raw samples through `commit` (interval=1 → one history row each).
fn fill(h: &mut History, n: u32, start_t: u32) -> u32 {
    for i in 0..n {
        h.commit(Sample {
            time_s: start_t + i,
            voltage: 13.0,
            battery_current: 1.0,
            ps_current: 2.0,
            power_online: 1.0,
        });
    }
    start_t + n
}

/// Directly push samples into the buffer, bypassing `commit`'s
/// accumulator. Used by max-interval tests that need a pre-filled
/// buffer at a chosen interval.
fn push_direct(h: &mut History, n: usize, start_t: u32) {
    for i in 0..n {
        assert!(
            h.samples
                .push(Sample {
                    time_s: start_t + i as u32,
                    voltage: 13.0,
                    battery_current: 1.0,
                    ps_current: 2.0,
                    power_online: 1.0,
                })
                .is_ok(),
            "history overflow"
        );
    }
}

#[test]
fn no_compaction_below_capacity() {
    let mut h = History::new();
    fill(&mut h, CAP as u32 - 1, 0);
    assert_eq!(h.samples.len(), CAP - 1);
    assert_eq!(h.interval, 1);
}

#[test]
fn compaction_at_capacity() {
    let mut h = History::new();
    fill(&mut h, CAP as u32 + 1, 0);
    assert_eq!(h.samples.len(), HALF + 1);
    assert_eq!(h.interval, 2);
}

#[test]
fn compaction_averages_all_fields_and_uses_later_timestamp() {
    let mut h = History::new();
    for i in 0..(CAP as u32 + 1) {
        let t = i * 10;
        let (v, c1, c2) = if i % 2 == 0 {
            (12.0, 1.0, 2.0)
        } else {
            (14.0, 3.0, 4.0)
        };
        h.commit(Sample {
            time_s: t,
            voltage: v,
            battery_current: c1,
            ps_current: c2,
            power_online: 1.0,
        });
    }
    assert_eq!(h.samples.len(), HALF + 1);
    assert_eq!(h.interval, 2);

    // Pairs (12,1,2) + (14,3,4) average to (13,2,3); all halves are
    // exact in f32 so equality is appropriate.
    for s in &h.samples[..HALF] {
        assert_eq!(s.voltage, 13.0);
        assert_eq!(s.battery_current, 2.0);
        assert_eq!(s.ps_current, 3.0);
    }
    assert_eq!(h.samples[0].time_s, 10);
    assert_eq!(h.samples[1].time_s, 30);
    assert_eq!(h.samples[HALF - 1].time_s, (CAP as u32 - 1) * 10);
}

#[test]
fn after_compaction_samples_at_new_interval() {
    let mut h = History::new();
    let t = fill(&mut h, CAP as u32 + 1, 0);
    assert_eq!(h.interval, 2);

    h.commit(Sample {
        time_s: t,
        voltage: 13.0,
        battery_current: 5.0,
        ps_current: 0.0,
        power_online: 1.0,
    });
    assert_eq!(h.samples.len(), HALF + 1);

    h.commit(Sample {
        time_s: t + 1,
        voltage: 13.0,
        battery_current: 7.0,
        ps_current: 0.0,
        power_online: 1.0,
    });
    assert_eq!(h.samples.len(), HALF + 2);
    let last = h.samples.last().unwrap();
    assert!(
        (last.battery_current - 6.0).abs() < 0.01,
        "battery_current {}, expected 6.0",
        last.battery_current
    );
    assert_eq!(last.time_s, t + 1);
}

#[test]
fn interval_doubles_each_compaction() {
    let mut h = History::new();
    assert_eq!(h.interval, 1);
    fill(&mut h, 820, 0);
    assert_eq!(h.interval, MAX_INTERVAL);
}

#[test]
fn long_run_stays_bounded_and_chronological() {
    let mut h = History::new();
    fill(&mut h, 10000, 0);
    assert!(h.samples.len() <= CAP);
    assert!(h.samples.len() >= HALF);
    for i in 1..h.samples.len() {
        assert!(
            h.samples[i].time_s >= h.samples[i - 1].time_s,
            "not chronological at {}: {} < {}",
            i,
            h.samples[i].time_s,
            h.samples[i - 1].time_s
        );
    }
}

#[test]
fn at_max_interval_drops_oldest_via_commit() {
    let mut h = History::new();
    h.interval = MAX_INTERVAL;
    push_direct(&mut h, CAP, 0);
    let oldest_before = h.samples[0].time_s;

    let base_t = 100_000;
    for i in 0..MAX_INTERVAL {
        h.commit(Sample {
            time_s: base_t + i,
            voltage: 13.0,
            battery_current: 5.0,
            ps_current: 3.0,
            power_online: 1.0,
        });
    }

    assert_eq!(h.samples.len(), CAP);
    assert_eq!(h.interval, MAX_INTERVAL);
    assert!(h.samples[0].time_s > oldest_before);
    let last_current = h.samples.last().unwrap().battery_current;
    assert!(
        (last_current - 5.0).abs() < 0.01,
        "battery_current {last_current}, expected 5.0"
    );
}

#[test]
fn transition_from_compaction_to_dropping() {
    let mut h = History::new();
    h.interval = MAX_INTERVAL / 2;
    push_direct(&mut h, CAP, 0);

    h.compact_if_needed();
    assert_eq!(h.samples.len(), HALF);
    assert_eq!(h.interval, MAX_INTERVAL);

    let first_after_compact = h.samples[0].time_s;
    push_direct(&mut h, HALF, CAP as u32);

    h.compact_if_needed();

    assert_eq!(h.samples.len(), CAP - 1);
    assert_eq!(h.interval, MAX_INTERVAL);
    assert!(h.samples[0].time_s > first_after_compact);
}

#[test]
#[should_panic(expected = "cannot average zero samples")]
fn sample_accum_average_panics_on_zero() {
    let acc = SampleAccum::default();
    acc.average(0, 0);
}
