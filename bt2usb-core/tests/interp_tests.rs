use bt2usb_core::interp::*;

// ============ Touch-point decode helpers ============

#[test]
fn decode_x_positive() {
    // 13-bit signed: max positive = 4095 (0x0FFF)
    // byte[0] = low 8 bits = 0xFF, byte[1] bits 0-4 = 0x0F
    let t = [0xFF, 0x0F, 0, 0, 0, 0, 0, 0, 0];
    assert_eq!(decode_x(&t), 4095);
}

#[test]
fn decode_x_negative() {
    // 13-bit signed: -1 = all 13 bits set
    // byte[0] = 0xFF, byte[1] bits 0-4 = 0x1F
    let t = [0xFF, 0x1F, 0, 0, 0, 0, 0, 0, 0];
    assert_eq!(decode_x(&t), -1);
}

#[test]
fn decode_x_zero() {
    let t = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    assert_eq!(decode_x(&t), 0);
}

#[test]
fn decode_x_small_positive() {
    // X = 100 (0x64): byte[0] = 0x64, byte[1] bits 0-4 = 0
    let t = [0x64, 0x00, 0, 0, 0, 0, 0, 0, 0];
    assert_eq!(decode_x(&t), 100);
}

#[test]
fn decode_y_zero() {
    let t = [0, 0, 0, 0, 0, 0, 0, 0, 0];
    assert_eq!(decode_y(&t), 0);
}

#[test]
fn finger_id_extraction() {
    let t = [0, 0, 0, 0, 0, 0, 0, 0, 0x05];
    assert_eq!(finger_id(&t), 5);
    // Upper nibble should be masked off
    let t2 = [0, 0, 0, 0, 0, 0, 0, 0, 0xF3];
    assert_eq!(finger_id(&t2), 3);
}

#[test]
fn touch_state_extraction() {
    // 0x80 = contact
    let t_contact = [0, 0, 0, 0x80, 0, 0, 0, 0, 0];
    assert_eq!(touch_state(&t_contact), 0x80);
    // 0x40 = near
    let t_near = [0, 0, 0, 0x40, 0, 0, 0, 0, 0];
    assert_eq!(touch_state(&t_near), 0x40);
    // 0xC0 = release
    let t_release = [0, 0, 0, 0xC0, 0, 0, 0, 0, 0];
    assert_eq!(touch_state(&t_release), 0xC0);
    // Other bits in byte[3] should be masked off
    let t_masked = [0, 0, 0, 0xBF, 0, 0, 0, 0, 0];
    assert_eq!(touch_state(&t_masked), 0x80);
}

// ============ Fixed-point conversion ============

#[test]
fn fixed_point_roundtrip() {
    for v in [-4096i16, -1, 0, 1, 100, 4095] {
        let fixed = to_fixed(v);
        let back = from_fixed(fixed);
        assert_eq!(back, v, "roundtrip failed for {}", v);
    }
}

#[test]
fn fixed_point_precision() {
    let fixed = to_fixed(100);
    assert_eq!(fixed, 100 << FRAC_BITS);
    assert_eq!(fixed, 100 * 4096);
}

// ============ InterpolationCore ============

fn make_bt_report(click: bool, touch_points: &[(i16, i16, u8, u8)]) -> Vec<u8> {
    // BT HIDP: [0x31][click_byte][hdr2][hdr3][touch×N]
    let mut data = Vec::new();
    data.push(0x31); // Report ID
    data.push(if click { 0x01 } else { 0x00 }); // Click byte
    data.push(0x00); // Header byte 2
    data.push(0x00); // Header byte 3

    for &(x, y, state, id) in touch_points {
        let mut point = [0u8; 9];
        // Encode X (13-bit signed): byte[0] = low 8, byte[1] bits 0-4 = high 5
        let x_raw = (x as u16) & 0x1FFF;
        point[0] = x_raw as u8;
        point[1] = ((x_raw >> 8) & 0x1F) as u8;

        // Encode Y (negated, stored across byte[1] bits 5-7, byte[2], byte[3] bits 0-1)
        let neg_y = (-y) as u16 & 0x1FFF;
        point[1] |= ((neg_y & 0x07) << 5) as u8;
        point[2] = ((neg_y >> 3) & 0xFF) as u8;
        point[3] = ((neg_y >> 11) & 0x03) as u8;

        // Touch state in byte[3] bits 6-7
        point[3] |= state;

        // Finger ID in byte[8] bits 0-3
        point[8] = id & 0x0F;

        data.extend_from_slice(&point);
    }
    data
}

#[test]
fn reject_invalid_report() {
    let mut core = InterpolationCore::new();
    // Too short
    assert!(core.receive_bt_report(&[0x31]).is_none());
    // Wrong report ID
    assert!(core.receive_bt_report(&[0x00, 0x00, 0x00, 0x00]).is_none());
    // Bad length (not multiple of 9 after header)
    assert!(core
        .receive_bt_report(&[0x31, 0, 0, 0, 1, 2, 3, 4, 5])
        .is_none());
}

#[test]
fn first_report_snaps_position() {
    let mut core = InterpolationCore::new();
    let report = make_bt_report(false, &[(100, 200, 0x80, 1)]);
    let info = core.receive_bt_report(&report).expect("valid report");
    assert_eq!(info.n_points, 1);
    assert!(!info.can_interp); // First report can't interpolate
    assert_eq!(info.count, 1);

    // First tick should output the snapped position
    let tick = core.tick().expect("should be active");
    assert_eq!(tick.n_fingers, 1);
    assert_eq!(tick.ids[0], 1);
    let out_x = from_fixed(tick.x[0]);
    let out_y = from_fixed(tick.y[0]);
    assert_eq!(out_x, 100);
    assert_eq!(out_y, 200);
}

#[test]
fn click_latching() {
    let mut core = InterpolationCore::new();

    // First report: no click
    let report = make_bt_report(false, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report);
    let tick = core.tick().unwrap();
    assert!(!tick.button);

    // Second report: click
    let report = make_bt_report(true, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report);
    let tick = core.tick().unwrap();
    assert!(tick.button, "click should be latched");

    // Third report: no click (released)
    let report = make_bt_report(false, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report);
    let tick = core.tick().unwrap();
    assert!(!tick.button, "click should be released");
}

#[test]
fn click_latch_persists_one_tick() {
    let mut core = InterpolationCore::new();

    // First report to initialize
    let report = make_bt_report(false, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report);
    core.tick();

    // Brief click: set and immediately clear before tick
    let report_click = make_bt_report(true, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report_click);
    let report_release = make_bt_report(false, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report_release);

    // Even though last_bt_click is false, latch should fire
    let tick = core.tick().unwrap();
    assert!(tick.button, "latched click should survive release");

    // After one tick, latch is consumed
    let tick2 = core.tick().unwrap();
    assert!(!tick2.button, "latch consumed after one tick");
}

#[test]
fn interpolation_between_reports() {
    let mut core = InterpolationCore::new();

    // First report at (0, 0)
    let report1 = make_bt_report(false, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report1);
    // Advance several ticks to establish timing
    for _ in 0..3 {
        core.tick();
    }

    // Second report at (300, 0) — should interpolate
    let report2 = make_bt_report(false, &[(300, 0, 0x80, 1)]);
    let info = core.receive_bt_report(&report2);
    assert!(
        info.as_ref().map_or(false, |i| i.can_interp),
        "should interpolate"
    );

    // Tick once — position should move toward target
    let tick = core.tick().unwrap();
    let x = from_fixed(tick.x[0]);
    assert!(x > 0 && x < 300, "position should be intermediate: {}", x);
}

#[test]
fn idle_timeout_deactivates() {
    let mut core = InterpolationCore::new();

    // Send a report with released finger (state 0xC0)
    let report = make_bt_report(false, &[(0, 0, 0xC0, 1)]);
    core.receive_bt_report(&report);

    // Tick many times past idle timeout (75 ticks + margin)
    for i in 0..100 {
        if core.tick().is_none() {
            // idle_ticks increments each tick(); deactivates when > 75
            // That means 76 increments = the 76th tick() call = i=75
            assert!(i >= 75, "should not deactivate before timeout (at i={})", i);
            return;
        }
    }
    panic!("should have deactivated by tick 100");
}

#[test]
fn active_fingers_prevent_idle_timeout() {
    let mut core = InterpolationCore::new();

    // Send a report with active finger (state 0x80 = contact)
    let report = make_bt_report(false, &[(0, 0, 0x80, 1)]);
    core.receive_bt_report(&report);

    // Tick many times — should NOT deactivate because finger is in contact
    for _ in 0..200 {
        assert!(
            core.tick().is_some(),
            "should stay active with contact finger"
        );
    }
}

#[test]
fn finger_count_change_resets_interp() {
    let mut core = InterpolationCore::new();

    // One finger
    let r1 = make_bt_report(false, &[(100, 100, 0x80, 1)]);
    core.receive_bt_report(&r1);
    core.tick();
    core.tick();
    core.tick();

    // Two fingers — different count, should snap not interpolate
    let r2 = make_bt_report(false, &[(200, 200, 0x80, 1), (300, 300, 0x80, 2)]);
    let info = core.receive_bt_report(&r2).unwrap();
    assert!(!info.can_interp, "finger count change should snap");
    assert_eq!(info.n_points, 2);
}

#[test]
fn empty_report_header_only() {
    let mut core = InterpolationCore::new();
    // Valid header with zero touch points
    let report = [0x31, 0x00, 0x00, 0x00];
    let info = core.receive_bt_report(&report).unwrap();
    assert_eq!(info.n_points, 0);
}
