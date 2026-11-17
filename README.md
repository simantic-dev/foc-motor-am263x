# foc-motor-am263x

Field-oriented control of a three-phase PMSM on a TI Sitara **AM2634** (Arm Cortex-R5F, 400 MHz),
built with TI's MCU+ SDK and tested on every commit in simulation with [Pyrite](https://simantic.com),
Simantic's firmware simulator. No board or motor is needed to build, run or test it.

- **The firmware** (`fw/foc.c`, NORTOS, TI driverlib): ePWM0/1/2 generate 20 kHz centre-aligned
  three-phase PWM; ePWM0 SOCA triggers ADC1 to sample both phase shunts at the centre of the
  low-side window; ADC1 INT1 (through INTXBAR) runs the 20 kHz current loop (Clarke, Park, PI on
  id and iq, inverse Park, SVPWM); a 1 kHz speed loop closes on eQEP1, a 500-line encoder. At boot
  it calibrates the current-sense offsets and aligns the rotor to find the encoder zero.
- **The simulation** (`sim/`): the firmware binary runs unmodified on the simulated AM2634.
  `plant.py` is a PMSM (R 0.2 Ohm, L 100 uH, 7 pole pairs, 24 V bus) that reads the ePWM compare
  registers and drives the ADC pins; `encoder.py` turns the shaft angle into eQEP A/B edges.
  The plant adds a 0.05 Nm load at 0.7 s.
- **The tests** (`tests/`): pytest runs the scenario and checks the console and the plant.

## Run it

```sh
make -C fw PROFILE=release MCU_PLUS_SDK_PATH=... CGT_TI_ARM_CLANG_PATH=... \
     SYSCFG_PATH=... SYSCFG_CLI_PATH=... foc.release.out   # run twice on a clean tree (SysConfig ordering)
sim run --scenario sim/motor.yaml --ascii --timeout 1.5
```

```
offsets  a=2048 b=2048 (LSB)
aligned  encoder zero = -25 counts
t= 100 ms ref= 1000 rpm speed=  923 rpm iq= 4969 mA isr=396 cycles
t= 280 ms ref= 1000 rpm speed= 1002 rpm iq=  -10 mA isr=395 cycles
t= 400 ms ref= 3000 rpm speed= 2430 rpm iq= 4981 mA isr=398 cycles
t= 640 ms ref= 3000 rpm speed= 3000 rpm iq= 1289 mA isr=397 cycles
isr worst 477 cycles = 1192 ns at 400 MHz (budget 50000 ns)
hold 1000 rpm: 5/5, hold 3000 rpm: 5/5
RESULT: PASS
[plant] plant t=1100 ms shaft=3000 rpm iq=1.28 A load=0.050 Nm
```

Needs TI MCU+ SDK for AM263x 26.00.00.06, TI Arm Clang 5.1.1 and SysConfig.

## Test it

```sh
pip install pytest && pytest tests -v     # SIM=<path> picks a sim binary
```

| Test | Checks |
| --- | --- |
| `test_boots_and_finishes` | firmware comes up and reports PASS |
| `test_current_sense_offsets_are_midscale` | ADC offset calibration |
| `test_holds_1000_rpm`, `test_holds_3000_rpm_under_load` | speed loop tracks its steps, through the load step |
| `test_plant_shaft_agrees_with_the_encoder` | the plant's true speed, not only the firmware's own view |
| `test_load_torque_is_carried_by_iq` | iq settles at load / Kt |
| `test_isr_fits_the_pwm_period` | current loop under 10% of the 50 us period (simulated cycle count) |

## Limits

- Simulated only: not yet run on an AM263x LaunchPad and motor. Cycle counts are the simulator's
  estimate, not a silicon measurement.
- The plant averages switching (no ripple, no dead time) and the sensors are ideal.
- Needs a `sim` build with the AM263x ADC volts input and ePWM peek (pyrite PR);
  the released 0.10.2 does not have them.
