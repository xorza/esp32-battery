/// LiFePO4 4S (12V nominal) voltage-to-SOC lookup table.
/// Each entry is (voltage_v, charge_percent).
const SOC_TABLE: &[(f32, f32)] = &[
    (10.00, 0.0),
    (10.16, 0.5),
    (11.20, 5.0),
    (12.00, 9.5),
    (12.20, 15.0),
    (12.80, 20.0),
    (12.92, 30.0),
    (13.00, 40.0),
    (13.04, 50.0),
    (13.12, 60.0),
    (13.20, 70.0),
    (13.32, 80.0),
    (13.40, 90.0),
    (13.52, 99.0),
    (13.80, 99.5),
    (14.60, 100.0),
];

/// Returns estimated charge percentage (0.0–100.0) for a given 4S LiFePO4 bus voltage.
/// Linearly interpolates between known data points.
pub fn ocv_soc(voltage_v: f32) -> f32 {
    if !voltage_v.is_finite() {
        return 0.0;
    }

    if voltage_v <= SOC_TABLE[0].0 {
        return SOC_TABLE[0].1;
    }
    if voltage_v >= SOC_TABLE[SOC_TABLE.len() - 1].0 {
        return SOC_TABLE[SOC_TABLE.len() - 1].1;
    }

    for i in 1..SOC_TABLE.len() {
        let (v_lo, soc_lo) = SOC_TABLE[i - 1];
        let (v_hi, soc_hi) = SOC_TABLE[i];
        if voltage_v <= v_hi {
            let t = (voltage_v - v_lo) / (v_hi - v_lo);
            return soc_lo + t * (soc_hi - soc_lo);
        }
    }

    unreachable!()
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn below_minimum_returns_zero() {
        assert_eq!(ocv_soc(5.0), 0.0);
        assert_eq!(ocv_soc(9.0), 0.0);
        assert_eq!(ocv_soc(10.0), 0.0);
    }

    #[test]
    fn above_maximum_returns_hundred() {
        assert_eq!(ocv_soc(14.60), 100.0);
        assert_eq!(ocv_soc(15.0), 100.0);
        assert_eq!(ocv_soc(20.0), 100.0);
    }

    #[test]
    fn exact_table_entries() {
        for &(v, soc) in SOC_TABLE {
            let result = ocv_soc(v);
            assert!(
                (result - soc).abs() < 0.01,
                "ocv_soc({v}) = {result}, expected {soc}"
            );
        }
    }

    #[test]
    fn interpolation_midpoint() {
        // Midpoint between (13.00, 40.0) and (13.04, 50.0)
        let result = ocv_soc(13.02);
        assert!(
            (result - 45.0).abs() < 0.1,
            "ocv_soc(13.02) = {result}, expected ~45.0"
        );
    }

    #[test]
    fn interpolation_quarter_point() {
        // Quarter point between (10.00, 0.0) and (10.16, 0.5)
        let result = ocv_soc(10.04);
        let expected = 0.0 + 0.25 * 0.5; // 0.125
        assert!(
            (result - expected).abs() < 0.01,
            "ocv_soc(10.04) = {result}, expected ~{expected}"
        );
    }

    #[test]
    fn monotonically_increasing() {
        let mut prev = ocv_soc(9.0);
        let mut v = 9.0;
        while v <= 15.0 {
            let soc = ocv_soc(v);
            assert!(
                soc >= prev,
                "SOC decreased: ocv_soc({v}) = {soc} < prev {prev}"
            );
            prev = soc;
            v += 0.01;
        }
    }

    #[test]
    fn output_range() {
        let mut v = 0.0;
        while v <= 20.0 {
            let soc = ocv_soc(v);
            assert!(
                (0.0..=100.0).contains(&soc),
                "ocv_soc({v}) = {soc} out of range"
            );
            v += 0.1;
        }
    }

    #[test]
    fn nan_returns_zero() {
        assert_eq!(ocv_soc(f32::NAN), 0.0);
    }

    #[test]
    fn infinity_returns_zero() {
        assert_eq!(ocv_soc(f32::INFINITY), 0.0);
    }

    #[test]
    fn neg_infinity_returns_zero() {
        assert_eq!(ocv_soc(f32::NEG_INFINITY), 0.0);
    }
}
