"""FOC of a PMSM on a simulated AM2634 R5F. No board is involved."""
CPU_HZ = 400_000_000
PWM_PERIOD_NS = 50_000


def test_boots_and_finishes(run):
    assert "== foc-motor-am263x: FOC of a PMSM on the AM2634 R5F ==" in run.console
    assert "RESULT: PASS" in run.console


def test_current_sense_offsets_are_midscale(run):
    m = run.field(r"offsets  a=(\d+) b=(\d+)")
    assert abs(int(m[1]) - 2048) < 10 and abs(int(m[2]) - 2048) < 10


def test_holds_1000_rpm(run):
    held = [r["rpm"] for r in run.rows if 200 <= r["ms"] < 300]
    assert held and all(abs(v - 1000) < 20 for v in held), held


def test_holds_3000_rpm_under_load(run):
    held = [r["rpm"] for r in run.rows if 500 <= r["ms"] < 900]
    assert held and all(abs(v - 3000) < 60 for v in held), held


def test_plant_shaft_agrees_with_the_encoder(run):
    """The firmware's speed comes from eQEP; the plant's is the truth."""
    loaded = [p for p in run.plant if p["load"] > 0 and 1000 <= p["ms"] <= 1300]
    assert loaded and all(abs(p["rpm"] - 3000) < 60 for p in loaded), loaded


def test_load_torque_is_carried_by_iq(run):
    """0.05 Nm / Kt 0.042 Nm/A = 1.19 A, plus friction."""
    iq = [p["iq"] for p in run.plant if p["load"] > 0 and 1000 <= p["ms"] <= 1300]
    assert iq and all(1.1 < v < 1.45 for v in iq), iq


def test_isr_fits_the_pwm_period(run):
    m = run.field(r"isr worst (\d+) cycles")
    assert int(m[1]) / CPU_HZ * 1e9 < PWM_PERIOD_NS / 10, "current loop must use under 10% of the period"
