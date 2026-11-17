/*
 * Field-oriented control of a PMSM on the AM2634 R5F (TI MCU+ SDK, NORTOS).
 *
 * ePWM0/1/2 drive phases A/B/C: 20 kHz up-down count, high side on while TBCTR < CMPA.
 * ePWM0 SOCA at TBCTR = PRD (centre of the low-side window) starts ADC1 SOC0 (phase A
 * shunt, AIN0) and SOC1 (phase B, AIN1); ADC1 INT1 at the end of SOC1 runs the current
 * loop through INTXBAR0. eQEP1 counts a 500-line encoder (2000 counts per turn).
 *
 * The ISR: Clarke, Park, two PI current loops, inverse Park, min-max SVPWM. Every 20th
 * pass (1 kHz) it runs the speed PI. The main loop prints telemetry and checks the result.
 */
#include <math.h>
#include <stdint.h>
#include <kernel/dpl/DebugP.h>
#include <kernel/dpl/HwiP.h>
#include <kernel/dpl/CycleCounterP.h>
#include <drivers/epwm.h>
#include <drivers/adc.h>
#include <drivers/eqep.h>
#include <drivers/soc.h>
#include "ti_drivers_config.h"
#include "ti_drivers_open_close.h"
#include "ti_board_open_close.h"

#define PI_F        3.14159265f
#define PWM_HZ      10000u
#define TBPRD       (200000000u / (2u * PWM_HZ))   /* 200 MHz TBCLK, up-down */
#define TS          (1.0f / PWM_HZ)
#define SPEED_DIV   20u                             /* speed loop at 1 kHz */
#define VBUS        24.0f
#define POLE_PAIRS  7
#define COUNTS      2000                            /* encoder counts per mechanical turn */
#define AMP_PER_LSB (3.3f / 4096.0f / 0.1f)         /* 10 mOhm shunt x10 amplifier: 0.1 V/A */
#define IQ_MAX      5.0f
#define TICKS(ms)   ((ms) * (PWM_HZ / 1000u))         /* ISR passes in ms */
#define RPM(w)      ((w) * 60.0f / (2.0f * PI_F))

static const uint32_t PWM[3] = { CSL_CONTROLSS_G0_EPWM0_U_BASE, CSL_CONTROLSS_G0_EPWM1_U_BASE,
                                 CSL_CONTROLSS_G0_EPWM2_U_BASE };
#define ADC   CONFIG_ADC1_BASE_ADDR
#define ADCR  CONFIG_ADC1_RESULT_BASE_ADDR
#define QEP   CSL_CONTROLSS_EQEP1_U_BASE

typedef struct { float kp, ki, i, lim; } Pi;

/* PI with conditional integration: the integrator holds while the output is saturated,
 * so a current-limited speed step does not wind it up and overshoot. */
static float pi_run(Pi *c, float err)
{
    float u = c->kp * err + c->i + c->ki * err;
    if (u > c->lim) return c->lim;
    if (u < -c->lim) return -c->lim;
    c->i += c->ki * err;
    return u;
}

/* Current loop at 1.5 kHz bandwidth for R = 0.2 Ohm, L = 100 uH; speed loop at 15 Hz for
 * J = 1e-4 kg m^2, Kt = 1.5 * 7 * 0.004 = 0.042 Nm/A. */
static Pi gId = { 0.94f, 1885.0f * TS, 0, VBUS / 1.7320508f };
static Pi gIq = { 0.94f, 1885.0f * TS, 0, VBUS / 1.7320508f };
static Pi gW  = { 0.224f, 5.3f / 1000.0f, 0, IQ_MAX };

static volatile uint32_t gTicks, gIsrCycles, gIsrMax;
static volatile float gRpmRef, gRpm, gIq_meas, gIdRef;
static volatile int gMode;                     /* 0 offset cal, 1 align, 2 closed loop */
static float gOffA, gOffB;
static int32_t gZero, gHist[5];
static int64_t gAlignSum;
static uint32_t gAlignN;
static HwiP_Object gHwi;

static void set_duty(const float d[3])
{
    for (int k = 0; k < 3; k++) {
        float x = d[k] < 0.02f ? 0.02f : (d[k] > 0.98f ? 0.98f : d[k]);
        EPWM_setCounterCompareValue(PWM[k], EPWM_COUNTER_COMPARE_A, (uint16_t)(x * TBPRD));
    }
}

static void foc_isr(void *arg)
{
    uint32_t t0 = CycleCounterP_getCount32();
    (void)arg;
    float ra = ADC_readResult(ADCR, ADC_SOC_NUMBER0), rb = ADC_readResult(ADCR, ADC_SOC_NUMBER1);
    ADC_clearInterruptStatus(ADC, ADC_INT_NUMBER1);
    uint32_t tick = ++gTicks;
    int32_t pos = (int32_t)EQEP_getPosition(QEP);

    if (gMode == 0) {                      /* 50% duty, zero current: learn the amplifier offsets */
        gOffA += ra; gOffB += rb;
        if (tick == TICKS(50)) { gOffA /= TICKS(50); gOffB /= TICKS(50); gMode = 1; }
        return;
    }
    float ia = (ra - gOffA) * AMP_PER_LSB, ib = (rb - gOffB) * AMP_PER_LSB;
    float ial = ia, ibe = (ia + 2.0f * ib) / 1.7320508f;          /* Clarke */

    float th = 0.0f;
    if (gMode == 2) {
        int32_t m = (pos - gZero) % COUNTS;
        th = (float)m * (2.0f * PI_F * POLE_PAIRS / COUNTS);
    }
    float c = cosf(th), s = sinf(th);
    float id = ial * c + ibe * s, iq = -ial * s + ibe * c;          /* Park */

    static float iqRef;
    if (gMode == 1) {                       /* hold 3 A on the d axis; zero = mean position */
        if (tick > TICKS(250)) { gAlignSum += pos; gAlignN++; }
        if (tick == TICKS(400)) {
            gZero = (int32_t)(gAlignSum / (int64_t)gAlignN);
            for (int k = 0; k < 5; k++) gHist[k] = pos;
            gIdRef = 0.0f; gId.i = 0.0f;
            gMode = 2;
        }
    } else if (tick % SPEED_DIV == 0) {     /* 1 kHz speed loop, speed over a 5 ms window */
        uint32_t k = (tick / SPEED_DIV) % 5;
        float w = (float)(pos - gHist[k]) * (2.0f * PI_F / COUNTS) / 0.005f;
        gHist[k] = pos;
        gRpm = RPM(w);
        iqRef = pi_run(&gW, (gRpmRef - gRpm) * (2.0f * PI_F / 60.0f));
        gIq_meas = iq;
    }
    float vd = pi_run(&gId, gIdRef - id), vq = pi_run(&gIq, iqRef - iq);

    float va = vd * c - vq * s, vb = vd * s + vq * c;               /* inverse Park */
    float v[3] = { va, -0.5f * va + 0.8660254f * vb, -0.5f * va - 0.8660254f * vb };
    float mx = fmaxf(v[0], fmaxf(v[1], v[2])), mn = fminf(v[0], fminf(v[1], v[2]));
    float off = -(mx + mn) * 0.5f, d[3];
    for (int k = 0; k < 3; k++) d[k] = 0.5f + (v[k] + off) / VBUS;
    set_duty(d);

    uint32_t dt = CycleCounterP_getCount32() - t0;
    gIsrCycles = dt;
    if (dt > gIsrMax && tick > TICKS(50)) gIsrMax = dt;
}

static void pwm_init(void)
{
    SOC_setMultipleEpwmTbClk(0x7, FALSE);          /* hold all three time bases */
    for (int k = 0; k < 3; k++) {
        uint32_t b = PWM[k];
        EPWM_setClockPrescaler(b, EPWM_CLOCK_DIVIDER_1, EPWM_HSCLOCK_DIVIDER_1);
        EPWM_setTimeBasePeriod(b, TBPRD);
        EPWM_setTimeBaseCounter(b, 0);
        EPWM_setTimeBaseCounterMode(b, EPWM_COUNTER_MODE_UP_DOWN);
        EPWM_setCounterCompareValue(b, EPWM_COUNTER_COMPARE_A, TBPRD / 2);
        EPWM_setActionQualifierAction(b, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_LOW, EPWM_AQ_OUTPUT_ON_TIMEBASE_UP_CMPA);
        EPWM_setActionQualifierAction(b, EPWM_AQ_OUTPUT_A, EPWM_AQ_OUTPUT_HIGH, EPWM_AQ_OUTPUT_ON_TIMEBASE_DOWN_CMPA);
    }
    EPWM_setADCTriggerSource(PWM[0], EPWM_SOC_A, EPWM_SOC_TBCTR_PERIOD, 0);
    EPWM_setADCTriggerEventPrescale(PWM[0], EPWM_SOC_A, 1);
    EPWM_enableADCTrigger(PWM[0], EPWM_SOC_A);
}

static void adc_init(void)
{
    ADC_setupSOC(ADC, ADC_SOC_NUMBER0, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN0, 16);
    ADC_setupSOC(ADC, ADC_SOC_NUMBER1, ADC_TRIGGER_EPWM0_SOCA, ADC_CH_ADCIN1, 16);
    ADC_setInterruptPulseMode(ADC, ADC_PULSE_END_OF_CONV);
    ADC_setInterruptSource(ADC, ADC_INT_NUMBER1, ADC_SOC_NUMBER1);
    ADC_enableInterrupt(ADC, ADC_INT_NUMBER1);
    ADC_clearInterruptStatus(ADC, ADC_INT_NUMBER1);
}

static void qep_init(void)
{
    EQEP_setDecoderConfig(QEP, EQEP_CONFIG_QUADRATURE | EQEP_CONFIG_2X_RESOLUTION | EQEP_CONFIG_NO_SWAP);
    EQEP_setEmulationMode(QEP, EQEP_EMULATIONMODE_RUNFREE);
    EQEP_setPositionCounterConfig(QEP, EQEP_POSITION_RESET_MAX_POS, 0xFFFFFFFFu);
    EQEP_enableModule(QEP);
}

/* Speed profile (ms since closed loop): 0 rpm, then 1000, then 3000; the plant adds a
 * load at its own 700 ms. */
static float profile(uint32_t ms) { return ms < 50 ? 0.0f : (ms < 300 ? 1000.0f : 3000.0f); }

void foc_main(void)
{
    Drivers_open();
    Board_driversOpen();
    DebugP_log("== foc-motor-am263x: FOC of a PMSM on the AM2634 R5F ==\r\n");
    DebugP_log("pwm %u Hz centre-aligned, TBPRD %u, current loop %u Hz, speed loop %u Hz\r\n",
               PWM_HZ, TBPRD, PWM_HZ, PWM_HZ / SPEED_DIV);

    CycleCounterP_reset();
    pwm_init();
    adc_init();
    qep_init();

    HwiP_Params p;
    HwiP_Params_init(&p);
    p.intNum = CSLR_R5FSS0_CORE0_CONTROLSS_INTRXBAR0_OUT_0;
    p.priority = 0;
    p.callback = foc_isr;
    DebugP_assert(HwiP_construct(&gHwi, &p) == SystemP_SUCCESS);
    SOC_setMultipleEpwmTbClk(0x7, TRUE);           /* start A, B, C together */

    while (gMode == 0) {}
    DebugP_log("offsets  a=%d b=%d (LSB)\r\n", (int)gOffA, (int)gOffB);
    gIdRef = 3.0f;
    while (gMode == 1) {}
    DebugP_log("aligned  encoder zero = %d counts\r\n", (int)gZero);

    uint32_t start = gTicks, last = 0, ok1 = 0, ok3 = 0, n1 = 0, n3 = 0;
    for (;;) {
        uint32_t ms = (gTicks - start) / (PWM_HZ / 1000);
        gRpmRef = profile(ms);
        if (ms >= last + 20) {
            last = ms - ms % 20;
            float rpm = gRpm, ref = gRpmRef;
            DebugP_log("t=%4u ms ref=%5d rpm speed=%5d rpm iq=%5d mA isr=%u cycles\r\n",
                       ms, (int)ref, (int)rpm, (int)(gIq_meas * 1000), gIsrCycles);
            if (ms >= 200 && ms < 300) { n1++; ok1 += fabsf(rpm - 1000) < 20; }
            if (ms >= 500 && ms < 600) { n3++; ok3 += fabsf(rpm - 3000) < 60; }
            if (ms >= 1100) break;
        }
    }
    uint32_t mhz = (uint32_t)(SOC_getSelfCpuClk() / 1000000u);
    DebugP_log("isr worst %u cycles = %u ns at %u MHz (budget %u ns)\r\n",
               gIsrMax, gIsrMax * 1000u / mhz, mhz, 1000000000u / PWM_HZ);
    DebugP_log("hold 1000 rpm: %u/%u, hold 3000 rpm: %u/%u\r\n", ok1, n1, ok3, n3);
    DebugP_log("RESULT: %s\r\n", (ok1 == n1 && ok3 == n3 && n1 && n3) ? "PASS" : "FAIL");
    DebugP_log("DONE\r\n");
    EPWM_setTimeBaseCounterMode(PWM[0], EPWM_COUNTER_MODE_STOP_FREEZE);
    for (;;) {}
}
