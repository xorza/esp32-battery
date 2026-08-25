use super::*;

use crate::data::history::{HISTORY_CAPACITY, internals::HistoryInternals};

fn bat_reading(voltage: f32, current: f32) -> Ina228Reading {
    Ina228Reading {
        voltage,
        current,
        power: voltage * current,
    }
}

fn ps_reading(voltage: f32, current: f32) -> PsReading {
    PsReading {
        voltage,
        current,
        power: voltage * current,
        v_set: 0.0,
        i_set: 0.0,
    }
}

/// Publish battery + PS readings and run one supervisor tick stamped with `now`.
fn update(sd: &mut SensorData, bat: Ina228Reading, p: PsReading, now: u32) {
    sd.update_battery(bat);
    sd.update_ps(p);
    sd.tick(Some(now));
}

/// Push n uniform samples (v=13, c1=1, c2=2). Returns the next time_s value.
fn fill(sd: &mut SensorData, n: u32, start_t: u32) -> u32 {
    for i in 0..n {
        update(
            sd,
            bat_reading(13.0, 1.0),
            ps_reading(13.0, 2.0),
            start_t + i,
        );
    }
    start_t + n
}

#[test]
fn default_is_empty() {
    let sd = SensorData::new();
    assert!(sd.history().is_empty());
    assert_eq!(sd.history.interval(), 1);
}

#[test]
fn single_update() {
    let mut sd = SensorData::new();
    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 100);

    assert_eq!(sd.history().len(), 1);
    let s = &sd.history()[0];
    assert_eq!(s.time_s, 100);
    assert!((s.voltage - 13.0).abs() < 0.001);
    assert!((s.battery_current - 1.0).abs() < 0.001);
    assert!((s.ps_current - 2.0).abs() < 0.001);
}

#[test]
fn voltage_from_battery_only() {
    let mut sd = SensorData::new();
    update(&mut sd, bat_reading(12.0, 1.0), ps_reading(14.0, 2.0), 1);
    assert!((sd.history()[0].voltage - 12.0).abs() < 0.001);
}

#[test]
fn latest_readings_visible_after_commit() {
    let mut sd = SensorData::new();
    update(&mut sd, bat_reading(13.0, 1.5), ps_reading(13.1, 2.5), 10);
    assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
    assert!((sd.ps_reading().unwrap().current - 2.5).abs() < 0.001);
}

#[test]
fn one_commit_per_tick_regardless_of_update_order() {
    let mut sd = SensorData::new();
    sd.update_ps(ps_reading(13.0, 2.0));
    sd.update_battery(bat_reading(13.0, 1.0));
    sd.update_ps(ps_reading(13.0, 2.0));
    sd.update_battery(bat_reading(13.0, 1.0));
    sd.tick(Some(1));
    assert_eq!(sd.history().len(), 1);

    sd.tick(Some(2));
    assert_eq!(sd.history().len(), 2);
}

#[test]
fn history_returns_all_entries() {
    let mut sd = SensorData::new();
    for i in 0..10u32 {
        update(
            &mut sd,
            bat_reading(13.0, i as f32),
            ps_reading(13.0, 0.0),
            i,
        );
    }
    let h = sd.history();
    assert_eq!(h.len(), 10);
    for (i, s) in h.iter().enumerate() {
        assert_eq!(s.time_s, i as u32);
        assert!((s.battery_current - i as f32).abs() < 0.001);
    }
}

#[test]
fn update_skipped_when_no_time() {
    let mut sd = SensorData::new();
    sd.update_ps(ps_reading(13.0, 2.0));
    sd.update_battery(bat_reading(13.0, 1.5));
    sd.tick(None);
    assert!(sd.history().is_empty());
    assert!((sd.battery_reading().unwrap().current - 1.5).abs() < 0.001);
    assert!((sd.ps_reading().unwrap().current - 2.0).abs() < 0.001);
}

#[test]
fn multiple_updates_accumulate() {
    let mut sd = SensorData::new();
    fill(&mut sd, 10, 0);
    assert_eq!(sd.history().len(), 10);
}

#[test]
fn power_online_threshold() {
    let mut sd = SensorData::new();
    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 0.0), 1);
    assert!((sd.history()[0].power_online - 1.0).abs() < 0.001);

    let mut sd = SensorData::new();
    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(1.0, 2.0), 1);
    assert!(sd.history()[0].power_online.abs() < 0.001);

    let mut sd = SensorData::new();
    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(0.0, 0.0), 1);
    assert!(sd.history()[0].power_online.abs() < 0.001);
}

#[test]
fn power_online_averaged_during_compaction() {
    let mut sd = SensorData::new();
    for i in 0..(HISTORY_CAPACITY as u32 + 1) {
        let v = if i % 2 == 0 { 13.0 } else { 0.0 };
        update(&mut sd, bat_reading(13.0, 1.0), ps_reading(v, 1.0), i);
    }
    assert_eq!(sd.history.interval(), 2);
    for s in &sd.history()[..HISTORY_CAPACITY / 2] {
        assert!((s.power_online - 0.5).abs() < 0.01);
    }
}

#[test]
fn out_of_order_commits_are_rejected() {
    let mut sd = SensorData::new();
    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2000);
    assert_eq!(sd.history().len(), 1);

    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 1500);
    assert_eq!(
        sd.history().len(),
        1,
        "backward-jump sample must not be pushed"
    );

    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2000);
    assert_eq!(sd.history().len(), 1);

    update(&mut sd, bat_reading(13.0, 1.0), ps_reading(13.0, 2.0), 2001);
    assert_eq!(sd.history().len(), 2);
}

#[test]
fn no_commit_before_ntp_sync() {
    let mut sd = SensorData::new();
    for _ in 0..100 {
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick(None);
    }
    assert!(sd.history().is_empty(), "no samples before NTP sync");
}

#[test]
fn battery_only_still_commits_with_ps_zeros() {
    let mut sd = SensorData::new();
    for i in 0..10u32 {
        sd.update_battery(bat_reading(13.0, 1.5));
        sd.tick(Some(100 + i));
    }
    assert_eq!(sd.history().len(), 10);
    for s in sd.history() {
        assert!((s.battery_current - 1.5).abs() < 0.001);
        assert!(s.ps_current.abs() < 0.001);
        assert!(s.power_online.abs() < 0.001);
    }
}

#[test]
fn ps_goes_stale_after_threshold() {
    // STALE_TICKS = 5 ticks of unrefreshed reading before the filter trips.
    const STALE: u32 = 5;
    let mut sd = SensorData::new();
    sd.update_battery(bat_reading(13.0, 1.0));
    sd.update_ps(ps_reading(13.0, 2.5));
    sd.tick(Some(1000));
    assert_eq!(sd.history().len(), 1);
    assert_eq!(sd.history()[0].ps_current, 2.5);
    assert_eq!(sd.history()[0].power_online, 1.0);
    let ps0 = sd.ps_reading().expect("ps fresh after first update");
    assert_eq!(ps0.current, 2.5);
    assert_eq!(ps0.voltage, 13.0);

    for i in 1..STALE {
        sd.update_battery(bat_reading(13.0, 1.0));
        sd.tick(Some(1000 + i));
    }
    let ps_late = sd.ps_reading().expect("PS still fresh at STALE_TICKS");
    assert_eq!(ps_late.current, 2.5);

    sd.update_battery(bat_reading(13.0, 1.0));
    sd.tick(Some(1000 + STALE));
    assert!(sd.ps_reading().is_none());
    let latest = sd.history().last().unwrap();
    assert_eq!(latest.ps_current, 0.0);
    assert_eq!(latest.power_online, 0.0);
}

#[test]
fn battery_stale_commits_zeros() {
    const STALE: u32 = 5;
    let mut sd = SensorData::new();
    sd.update_battery(bat_reading(13.0, 1.0));
    sd.update_ps(ps_reading(13.0, 2.0));
    sd.tick(Some(2000));
    assert_eq!(sd.history().len(), 1);
    assert_eq!(sd.history()[0].voltage, 13.0);

    let ticks = STALE + 3;
    for i in 1..=ticks {
        sd.update_ps(ps_reading(13.0, 2.0));
        sd.tick(Some(2000 + i));
    }
    assert_eq!(sd.history().len(), 1 + ticks as usize);
    assert!(sd.battery_reading().is_none());
    let latest = sd.history().last().unwrap();
    assert_eq!(latest.voltage, 0.0);
    assert_eq!(latest.battery_current, 0.0);
    assert_eq!(latest.ps_current, 2.0);
}
