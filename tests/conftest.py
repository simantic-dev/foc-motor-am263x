"""Runs the firmware once on a simulated AM2634 (Pyrite's `sim`) with the motor plant, and hands
the tests its console and the plant's log."""
import os, pathlib, re, shutil, subprocess
import pytest

ROOT = pathlib.Path(__file__).resolve().parent.parent


class Run:
    def __init__(self, text):
        self.console = text
        self.rows = [dict(ms=int(m[1]), ref=int(m[2]), rpm=int(m[3]), iq_ma=int(m[4]))
                     for m in re.finditer(r"t=\s*(\d+) ms ref=\s*(-?\d+) rpm speed=\s*(-?\d+) rpm iq=\s*(-?\d+) mA", text)]
        self.plant = [dict(ms=int(m[1]), rpm=int(m[2]), iq=float(m[3]), load=float(m[4]))
                      for m in re.finditer(r"plant t=(\d+) ms shaft=(-?\d+) rpm iq=(-?[\d.]+) A load=([\d.]+)", text)]

    def field(self, pattern):
        m = re.search(pattern, self.console)
        assert m, f"console has no {pattern!r}:\n{self.console[-2000:]}"
        return m


@pytest.fixture(scope="session")
def run():
    sim = os.environ.get("SIM") or shutil.which("sim")
    assert sim, "no `sim` on PATH: install it (https://simantic.com/install) or set SIM=<path>"
    assert (ROOT / "fw/foc.release.out").exists(), "firmware not built: see README"
    p = subprocess.run([sim, "run", "--scenario", str(ROOT / "sim/motor.yaml"), "--ascii", "--timeout", "1.5"],
                       capture_output=True, text=True, timeout=300)
    assert p.returncode == 0, f"sim failed ({p.returncode}):\n{p.stdout}\n{p.stderr}"
    return Run((p.stdout + p.stderr).replace("\r", ""))
