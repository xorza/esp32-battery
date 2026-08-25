use super::*;
use strum::IntoEnumIterator;

#[test]
fn new_is_empty() {
    let log = EventLog::new();
    assert!(log.is_empty());
    assert_eq!(log.len(), 0);
    for k in InaError::iter() {
        assert_eq!(log.ina_count(k), 0);
    }
    for k in XyError::iter() {
        assert_eq!(log.xy_count(k), 0);
    }
    for k in ChargeTransition::iter() {
        assert_eq!(log.charge_count(k), 0);
    }
}

#[test]
fn record_bumps_counter_and_appends() {
    let mut log = EventLog::new();
    log.record(100, Event::Ina(InaError::CurrentRead));
    assert_eq!(log.len(), 1);
    assert_eq!(log.ina_count(InaError::CurrentRead), 1);
    assert_eq!(log.ina_count(InaError::Init), 0);
    let e = log.recent().next().unwrap();
    assert_eq!(e.ts, 100);
    assert!(matches!(e.event, Event::Ina(InaError::CurrentRead)));
}

#[test]
fn distinct_kinds_within_source_counted_separately() {
    let mut log = EventLog::new();
    log.record(1, Event::Ina(InaError::CurrentRead));
    log.record(2, Event::Ina(InaError::CurrentRead));
    log.record(3, Event::Ina(InaError::PowerRead));
    assert_eq!(log.ina_count(InaError::CurrentRead), 2);
    assert_eq!(log.ina_count(InaError::PowerRead), 1);
    assert_eq!(log.ina_count(InaError::BusVoltageRead), 0);
}

#[test]
fn counters_are_independent_per_source() {
    let mut log = EventLog::new();
    log.record(1, Event::Ina(InaError::CurrentRead));
    log.record(2, Event::Xy(XyError::ReadStatus));
    log.record(3, Event::Xy(XyError::ReadStatus));
    log.record(4, Event::Charge(ChargeTransition::Energised));
    log.record(5, Event::Charge(ChargeTransition::Energised));
    log.record(6, Event::Charge(ChargeTransition::Latched));
    assert_eq!(log.ina_count(InaError::CurrentRead), 1);
    assert_eq!(log.xy_count(XyError::ReadStatus), 2);
    assert_eq!(log.charge_count(ChargeTransition::Energised), 2);
    assert_eq!(log.charge_count(ChargeTransition::Latched), 1);
    // A kind that was never recorded stays at zero across sources.
    assert_eq!(log.charge_count(ChargeTransition::ProtectHold), 0);
}

#[test]
fn iteration_order_is_oldest_first() {
    let mut log = EventLog::new();
    for ts in 1..=5 {
        log.record(ts, Event::Xy(XyError::ReadStatus));
    }
    let timestamps: heapless::Vec<u32, 5> = log.recent().map(|e| e.ts).collect();
    assert_eq!(&timestamps[..], &[1, 2, 3, 4, 5]);
}

#[test]
fn ring_evicts_oldest_when_full() {
    let mut log = EventLog::new();
    // Fill capacity + 5 — first 5 must be evicted.
    for ts in 0..(CAPACITY as u32 + 5) {
        log.record(ts, Event::Xy(XyError::ReadStatus));
    }
    assert_eq!(log.len(), CAPACITY);
    let first_ts = log.recent().next().unwrap().ts;
    let last_ts = log.recent().last().unwrap().ts;
    assert_eq!(first_ts, 5);
    assert_eq!(last_ts, CAPACITY as u32 + 4);
}

#[test]
fn counters_survive_ring_eviction() {
    // Counter is the lifetime total — ring overflow must not lose it.
    let mut log = EventLog::new();
    let pushes = CAPACITY as u32 + 17;
    for _ in 0..pushes {
        log.record(1, Event::Xy(XyError::ReadStatus));
    }
    assert_eq!(log.len(), CAPACITY);
    assert_eq!(log.xy_count(XyError::ReadStatus), pushes);
}

#[test]
fn pre_ntp_zero_timestamp_is_valid() {
    let mut log = EventLog::new();
    log.record(0, Event::Ina(InaError::Init));
    let e = log.recent().next().unwrap();
    assert_eq!(e.ts, 0);
    assert_eq!(log.ina_count(InaError::Init), 1);
}

#[test]
fn names_are_unique_across_sources() {
    let mut seen: heapless::Vec<
        &str,
        {
            <InaError as EnumCount>::COUNT
                + <XyError as EnumCount>::COUNT
                + <ChargeTransition as EnumCount>::COUNT
        },
    > = heapless::Vec::new();
    for k in InaError::iter() {
        assert!(seen.push(k.name()).is_ok());
    }
    for k in XyError::iter() {
        assert!(seen.push(k.name()).is_ok());
    }
    for k in ChargeTransition::iter() {
        assert!(seen.push(k.name()).is_ok());
    }
    for i in 0..seen.len() {
        for j in (i + 1)..seen.len() {
            assert_ne!(seen[i], seen[j], "duplicate name at {i}/{j}");
        }
    }
}

#[test]
fn indices_are_dense_and_match_count() {
    let mut ina_seen = [false; <InaError as EnumCount>::COUNT];
    for k in InaError::iter() {
        assert!(!ina_seen[k.index()], "duplicate index {}", k.index());
        ina_seen[k.index()] = true;
    }
    assert!(ina_seen.iter().all(|&b| b));

    let mut xy_seen = [false; <XyError as EnumCount>::COUNT];
    for k in XyError::iter() {
        assert!(!xy_seen[k.index()], "duplicate index {}", k.index());
        xy_seen[k.index()] = true;
    }
    assert!(xy_seen.iter().all(|&b| b));

    let mut charge_seen = [false; <ChargeTransition as EnumCount>::COUNT];
    for k in ChargeTransition::iter() {
        assert!(!charge_seen[k.index()], "duplicate index {}", k.index());
        charge_seen[k.index()] = true;
    }
    assert!(charge_seen.iter().all(|&b| b));
}

#[test]
fn from_protection_maps_every_cause() {
    // Normal is not a fault.
    assert_eq!(XyError::from_protection(ProtectionStatus::Normal), None);
    // Each fault cause maps to its own kind, names are cause-tagged.
    let cases = [
        (ProtectionStatus::Ovp, XyError::ProtectOvp, "protect_ovp"),
        (ProtectionStatus::Ocp, XyError::ProtectOcp, "protect_ocp"),
        (ProtectionStatus::Opp, XyError::ProtectOpp, "protect_opp"),
        (ProtectionStatus::Lvp, XyError::ProtectLvp, "protect_lvp"),
        (ProtectionStatus::Oah, XyError::ProtectOah, "protect_oah"),
        (ProtectionStatus::Ohp, XyError::ProtectOhp, "protect_ohp"),
        (ProtectionStatus::Otp, XyError::ProtectOtp, "protect_otp"),
        (ProtectionStatus::Oep, XyError::ProtectOep, "protect_oep"),
        (ProtectionStatus::Owh, XyError::ProtectOwh, "protect_owh"),
        (ProtectionStatus::Icp, XyError::ProtectIcp, "protect_icp"),
    ];
    for (status, expected, name) in cases {
        assert_eq!(XyError::from_protection(status), Some(expected));
        assert_eq!(expected.name(), name);
    }
}

#[test]
fn mixed_workload_keeps_each_source_consistent() {
    let mut log = EventLog::new();
    for i in 0..50 {
        match i % 3 {
            0 => log.record(i, Event::Ina(InaError::CurrentRead)),
            1 => log.record(i, Event::Xy(XyError::ReadStatus)),
            _ => log.record(i, Event::Charge(ChargeTransition::ProtectHold)),
        }
    }
    // 0,3,…,48 → 17 INA. 1,4,…,49 → 17 XY. 2,5,…,47 → 16 charge. 50 total.
    assert_eq!(log.ina_count(InaError::CurrentRead), 17);
    assert_eq!(log.xy_count(XyError::ReadStatus), 17);
    assert_eq!(log.charge_count(ChargeTransition::ProtectHold), 16);
    assert_eq!(log.len(), CAPACITY);
}
