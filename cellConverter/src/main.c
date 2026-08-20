#include "stm32g0xx_hal.h"
#include "stm32g0xx_hal_dac.h"
#include "stm32g0xx_hal_adc.h"
#include "stm32g0xx_hal_adc_ex.h"
#include "stm32g0xx_hal_pwr.h"
#include "stm32g0xx_hal_pwr_ex.h"
#include "stm32g0xx_hal_flash.h"
#include "stm32g0xx_hal_flash_ex.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdarg.h>

/* GPIO */
#define WAKE_PIN         GPIO_PIN_0   /* PA0 — optional external wake (WKUP1); see ENABLE_AUTO_SHUTDOWN below */
#define WAKE_PORT        GPIOA
#define CELL_PWR_PIN     GPIO_PIN_1   /* PA1 — cell MOSFET switch, active low (low = cell powered on) */
#define CELL_PWR_PORT    GPIOA

/*
 * USART2 — debug text output, TX-only: PA2 = USART2_TX, AF1.
 * The G051's Cortex-M0+ core has no ITM/TPIU, so SWO/ITM printf tracing
 * (which would otherwise ride the SWD connector for free) isn't available
 * on this chip — a real UART is the only option. PA2/PA3 are the former
 * fault/cell LED pins, now free; only TX is wired up since nothing needs
 * to be received. View with any 115200-8N1 USB-serial adapter/terminal, or
 * through a debug probe's VCP bridge (e.g. ST-Link/V2-1, V3) wired to PA2.
 */
#define DEBUG_TX_PIN     GPIO_PIN_2   /* PA2 */
#define DEBUG_PORT       GPIOA
#define DEBUG_BAUD       115200

/* USART1 — FDO2 sensor: PB6=TX, PB7=RX, AF0 */
#define FDO2_TX_PIN      GPIO_PIN_6
#define FDO2_RX_PIN      GPIO_PIN_7
#define FDO2_PORT        GPIOB
#define FDO2_BAUD        19200

/* DAC — PA4 = DAC1_OUT1 */
#define DAC_PIN          GPIO_PIN_4

/* Timing */
#define POLL_MS          500
#define RX_TIMEOUT_MS    500
/*
 * #MRAW replies carry 8 signed/unsigned 32-bit fields ("O T S D I A P H")
 * plus the "#MRAW " prefix; worst case (all fields near INT32_MIN, 11 chars
 * each) is ~102 bytes. 128 gives headroom without the risk #MOXY's old
 * 64-byte buffer would have of silently truncating the P (pressure) field.
 */
#define RX_BUF           128

/*
 * Power-on self-test DAC sweep
 *   POST_STEPS × POST_STEP_MS × 2 ≈ 3060 ms — satisfies FDO2 ≥1 s startup
 *   requirement with the same 3 s margin as the original delay.
 *   DAC_MAX (4095) corresponds to ~3.18 bar pO2 — the new 20k/1k divider's
 *   ceiling, comfortably above the ~2.6 bar operating target that motivated
 *   the resistor change from 33k/1k.
 */
#define POST_STEPS       50U   /* steps each way */
#define POST_STEP_MS     30U   /* ms per step → 51 × 30 × 2 = 3060 ms total */

/*
 * Safety limits
 *   PPO2_MAX_HPA    : pO2 above 2.6 bar → DAC immediately driven to 0. This
 *                     is a fault check on the raw FDO2 reading (out-of-range
 *                     / implausible data), independent of the DAC's own
 *                     ~3.18 bar hardware ceiling above.
 *   COMM_HOLD_CYCLES: hold last good DAC value for this many consecutive comm
 *                     faults, then drive to 0 on the next failure.
 *   VCC_MIN_MV      : if VCC falls below this threshold the FDO2 can give
 *                     false readings; DAC is driven to 0. Checked during
 *                     POST and before every sensor poll.
 */
#define PPO2_MAX_HPA     2600U    /* 2.6 bar expressed in hPa */
#define COMM_HOLD_CYCLES 3U
#define VCC_MIN_MV       3250U    /* 3.25 V minimum supply */

/*
 * Auto-shutdown (optional) — PA0 / WKUP1
 *   The board can power itself down to STM32 Standby mode (the lowest-power
 *   state the G051 supports) whenever the FDO2 reports the unit is at the
 *   surface and not reading a dangerously high pO2. PA0 is the hardware
 *   WKUP1 pin: driving it high wakes the chip, which restarts execution from
 *   the top of main() — a Standby wake is indistinguishable from a full
 *   reset (RM0444).
 *
 *   ENABLE_AUTO_SHUTDOWN is a compile-time switch: while 0 (default), PA0
 *   and all of the logic below are inert and the board just runs
 *   continuously, matching pre-Standby-feature behaviour. Flip to 1 only
 *   after confirming PA0 wiring and these thresholds on the bench.
 *
 *   The condition is checked every poll cycle, but only on a *successful*
 *   reading — never on a comm fault or low-VCC condition — so the unit
 *   fails safe (stays awake) whenever it can't currently trust its own
 *   sensor data.
 *     SURFACE_PRESSURE_HPA : ambient pressure (FDO2 #MRAW "P" field) below
 *                             this is considered "at the surface". Standard
 *                             sea level is 1013 hPa and each metre of depth
 *                             adds roughly 100 hPa, so this has margin above
 *                             ordinary weather-driven variation.
 *     PPO2_SHUTDOWN_HPA    : pO2 above this keeps the unit awake regardless
 *                             of pressure — a high reading while shallow can
 *                             still mean active use.
 */
#define ENABLE_AUTO_SHUTDOWN   0U        /* 1 to enable Standby / wake-pin behaviour */
#define SURFACE_PRESSURE_HPA   1050U
#define PPO2_SHUTDOWN_HPA      950U      /* 0.95 bar */

/*
 * IWDG — Independent Watchdog
 *   LSI ~32 kHz, prescaler /64 → 2 ms per tick.
 *   Reload 2000 → ~4 s nominal timeout.
 *   With ±40 % LSI tolerance: worst-case minimum ~2.4 s.
 *   Max legitimate loop iteration: 500 ms poll delay + 500 ms RX timeout
 *   + 100 ms worst-case DebugLog() UART timeout ≈ 1100 ms — well inside
 *   2.4 s minimum.
 *   Watchdog is kicked once per main-loop iteration; any firmware lockup
 *   (hung HAL call, infinite loop, stray execution) will fire a reset.
 *
 *   The IWDG keeps counting during Standby mode — it's clocked by the
 *   independent LSI oscillator, which Standby does not stop — so without
 *   further action it would reset the chip every ~4 s while "asleep",
 *   defeating ENABLE_AUTO_SHUTDOWN entirely. EnsureIwdgStandbyFreeze()
 *   below self-provisions the IWDG_STDBY flash option byte on first boot to
 *   freeze the counter for the duration of Standby.
 */
#define IWDG_PRESC       IWDG_PRESCALER_64
#define IWDG_RELOAD_VAL  2000U

/*
 * DAC scaling: MD22 cell — 50 mV at 1013 hPa pure O2 (1 ata), linear
 *   Divider  V_cell = V_dac / 21  (R1=20k, R2=1k)
 *   V_dac    = 50 mV * 21 * pO2_hPa / 1013
 *   DAC count = V_dac / 3300 mV * 4095
 *   Simplifies to: count = pO2_hPa * 1286 / 1000
 *   At air     (213 hPa, 21% O2)            → V_dac ≈ 220.7 mV
 *   At 0.7 bar (700 hPa)                    → V_dac ≈ 725.6 mV
 *   At 1.0 bar (1000 hPa)                   → V_dac ≈ 1036.5 mV
 *   At 1.3 bar (1300 hPa)                   → V_dac ≈ 1347.5 mV
 *   At 2.6 bar (2600 hPa, new PPO2_MAX_HPA) → V_dac ≈ 2694.0 mV
 */
#define PSR_SCALE_NUM    1286U
#define PSR_SCALE_DEN    1000U
#define DAC_MAX          4095U

/*
 * ADC — internal VCC measurement via VREFINT
 *   The STM32G051 stores a factory VREFINT calibration value at 0x1FFF75AA
 *   (measured at 3.0 V).  __HAL_ADC_CALC_VREFANALOG_VOLTAGE() uses this to
 *   back-calculate the actual VCC from a VREFINT ADC reading.
 *   ADC clock = PCLK/2 = 8 MHz; 160.5-cycle sampling → 20 µs per sample,
 *   satisfying the VREFINT ≥4 µs requirement with generous margin.
 */
#define ADC_PRESC        ADC_CLOCK_SYNC_PCLK_DIV2
#define ADC_SAMP_TIME    ADC_SAMPLETIME_160CYCLES_5

static UART_HandleTypeDef huart1;
static UART_HandleTypeDef huart2;
static DAC_HandleTypeDef  hdac;
static IWDG_HandleTypeDef hiwdg;
static ADC_HandleTypeDef  hadc;

static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_USART1_Init(void);
static void MX_USART2_Init(void);
static void DebugLog(const char *fmt, ...);
static void MX_DAC_Init(void);
static void MX_IWDG_Init(void);
static void MX_ADC_Init(void);
static void Fatal_Init_Fault(void);
static void SleepMs(uint32_t ms);
static uint32_t ReadVCC_mV(void);
static int      CheckVCC(void);
static void POST_DacSweep(void);
static void FDO2_FlushRx(void);
static HAL_StatusTypeDef FDO2_ReadLine(char *buf, uint16_t maxLen);
static void FDO2_Poll(void);
static void EnsureIwdgStandbyFreeze(void);
static void EnterAutoStandby(void);

/* Consecutive comm-fault counter — persists across poll cycles */
static uint32_t consec_comm_fails = 0;

int main(void)
{
    HAL_Init();

    /*
     * GPIO first so the cell MOSFET (PA1) is driven to a known state before
     * any subsequent init call can fail.
     */
    MX_GPIO_Init();
    SystemClock_Config();
    MX_USART2_Init();
    DebugLog("\r\n--- cellConverter boot ---\r\n");

    /*
     * Must run before MX_IWDG_Init(): if the IWDG_STDBY option byte isn't
     * already frozen, this reprograms it and resets the MCU (only happens
     * once, on whichever boot first sees the mismatch).
     */
    if (ENABLE_AUTO_SHUTDOWN)
        EnsureIwdgStandbyFreeze();

    /* Watchdog running — every subsequent operation must complete before expiry */
    MX_IWDG_Init();

    MX_DAC_Init();
    MX_USART1_Init();
    MX_ADC_Init();

    /*
     * Power-on self-test: VCC check then DAC sweep so the operator can verify
     * the supply rail and output chain before entering service.
     * Also serves as the FDO2 power-up delay (~3 s total, minimum 1 s).
     */
    POST_DacSweep();

    while (1)
    {
        /* Prove main loop alive; must reach here within watchdog timeout */
        HAL_IWDG_Refresh(&hiwdg);
        FDO2_Poll();
        SleepMs(POLL_MS);
    }
}

static void FDO2_Poll(void)
{
    char     rx[RX_BUF];
    char    *p;
    char    *end;
    long     O;
    long     T;
    unsigned long S;
    long     tmp;
    long     P;
    uint32_t pO2_hPa;
    uint32_t pressure_hPa;
    uint32_t dac_val;

    /*
     * VCC check before every sensor poll.
     * If supply is marginal the FDO2 can produce false readings; driving the
     * DAC to 0 is safer than forwarding a potentially invalid pO2 value.
     * Recovery is automatic: the main loop keeps calling FDO2_Poll(), so a
     * restored supply immediately resumes normal operation.
     */
    if (!CheckVCC())
        return;

    FDO2_FlushRx();

    if (HAL_UART_Transmit(&huart1, (uint8_t *)"#MRAW\r", 6, 50) != HAL_OK)
        goto comm_fault;

    if (FDO2_ReadLine(rx, sizeof(rx)) != HAL_OK)
        goto comm_fault;

    if (strncmp(rx, "#MRAW ", 6) != 0)
        goto comm_fault;

    /*
     * Fixed field order per the FDO2-G2 datasheet, #MRAW command:
     *   O T S D I A P H
     * O/T/S match #MOXY. D (dphi), I (intensity) and A (ambient light) are
     * parsed and discarded — only P (ambient pressure, signed, µbar) is
     * needed here, to detect surface vs. submerged for auto-shutdown.
     */
    p = rx + 6;
    O = strtol(p, &end, 10);
    if (end == p) goto comm_fault;   /* no digits consumed */
    p = end;
    T = strtol(p, &end, 10);
    if (end == p) goto comm_fault;
    p = end;
    S = strtoul(p, &end, 10);
    if (end == p) goto comm_fault;
    p = end;
    (void)T;

    tmp = strtol(p, &end, 10);       /* D */
    if (end == p) goto comm_fault;
    p = end;
    tmp = strtol(p, &end, 10);       /* I */
    if (end == p) goto comm_fault;
    p = end;
    tmp = strtol(p, &end, 10);       /* A */
    if (end == p) goto comm_fault;
    p = end;
    (void)tmp;

    P = strtol(p, &end, 10);         /* P */
    if (end == p) goto comm_fault;

    /* bit 0 = warning (reading still valid); bits 1+ = sensor error */
    if (S & ~0x01UL)
        goto comm_fault;

    /*
     * pO2 above 2.6 bar: drive DAC to 0 immediately.
     * This is not a comm fault — comms are healthy — so the hold counter
     * is reset and the output is zeroed on every out-of-range reading.
     * Recovery is automatic: a subsequent in-range reading restores the DAC.
     */
    if (O < 0 || O > (long)(PPO2_MAX_HPA * 1000UL))
    {
        DebugLog("pO2 OUT OF RANGE (raw=%ld) - DAC forced to 0\r\n", O);
        consec_comm_fails = 0;
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
        return;
    }

    if (P < 0)
        goto comm_fault;   /* implausible ambient pressure reading */

    pO2_hPa      = (uint32_t)((O + 500L) / 1000L);
    pressure_hPa = (uint32_t)((P + 500L) / 1000L);
    dac_val      = pO2_hPa * PSR_SCALE_NUM / PSR_SCALE_DEN;
    if (dac_val > DAC_MAX) dac_val = DAC_MAX;

    consec_comm_fails = 0;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, dac_val);

    DebugLog("pO2=%lu hPa  P=%lu hPa  DAC=%lu\r\n",
             (unsigned long)pO2_hPa, (unsigned long)pressure_hPa, (unsigned long)dac_val);

    /*
     * Auto-shutdown: only ever evaluated on a successful, trusted reading
     * (never on a comm fault or low-VCC return above), so the unit fails
     * safe by staying awake whenever it can't currently trust its data.
     */
    if (ENABLE_AUTO_SHUTDOWN &&
        pressure_hPa < SURFACE_PRESSURE_HPA &&
        pO2_hPa <= PPO2_SHUTDOWN_HPA)
    {
        DebugLog("Surface + low ppO2 -> entering Standby\r\n");
        EnterAutoStandby();   /* does not return */
    }
    return;

comm_fault:
    /*
     * Hold last DAC value for the first COMM_HOLD_CYCLES failures so brief
     * dropouts don't immediately zero the output.  After that, drive DAC to 0
     * so the host rebreather sees a clear sensor-absent indication.
     * The main loop keeps calling FDO2_Poll(), so recovery is automatic once
     * comms are restored.
     */
    if (consec_comm_fails <= COMM_HOLD_CYCLES)
        consec_comm_fails++;

    if (consec_comm_fails > COMM_HOLD_CYCLES)
    {
        DebugLog("FDO2 comm fault (%lu/%lu) - DAC forced to 0\r\n",
                 (unsigned long)consec_comm_fails, (unsigned long)COMM_HOLD_CYCLES);
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    }
    else
    {
        DebugLog("FDO2 comm fault (%lu/%lu) - holding last value\r\n",
                 (unsigned long)consec_comm_fails, (unsigned long)COMM_HOLD_CYCLES);
    }
}

/*
 * Returns VCC in mV, calculated from the internal VREFINT measurement.
 * Uses the factory calibration word at 0x1FFF75AA (taken at 3.0 V) via the
 * HAL macro so that no hard-coded constants are needed here.
 */
static uint32_t ReadVCC_mV(void)
{
    ADC_ChannelConfTypeDef sConfig = {0};
    uint32_t raw;

    sConfig.Channel      = ADC_CHANNEL_VREFINT;
    sConfig.Rank         = ADC_RANK_CHANNEL_NUMBER;
    sConfig.SamplingTime = ADC_SAMPLINGTIME_COMMON_1;
    HAL_ADC_ConfigChannel(&hadc, &sConfig);

    HAL_ADC_Start(&hadc);
    HAL_ADC_PollForConversion(&hadc, 10);
    raw = HAL_ADC_GetValue(&hadc);
    HAL_ADC_Stop(&hadc);

    return __HAL_ADC_CALC_VREFANALOG_VOLTAGE(raw, ADC_RESOLUTION_12B);
}

/*
 * Returns 1 if VCC is at or above VCC_MIN_MV.
 * On low VCC: drives DAC to 0, resets the comm-fault counter (any readings
 * taken under low VCC are suspect and should not be held).
 * Recovery is automatic on the next successful call.
 */
static int CheckVCC(void)
{
    uint32_t vcc = ReadVCC_mV();

    if (vcc >= VCC_MIN_MV)
        return 1;

    DebugLog("VCC low: %lu mV - DAC forced to 0\r\n", (unsigned long)vcc);
    consec_comm_fails = 0;
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
    return 0;
}

static void POST_DacSweep(void)
{
    /*
     * Ceiling division ensures the final step reaches exactly DAC_MAX:
     * 4095 / 50 = 81 remainder 45 → ceil = 82; step 50 × 82 = 4100, capped.
     */
    const uint32_t step_size = (DAC_MAX + POST_STEPS - 1U) / POST_STEPS;
    uint32_t s;
    uint32_t val;

    /*
     * VCC must be healthy before sweeping.  If the supply is already low at
     * power-on, skip the sweep but still wait the full FDO2 startup time so
     * comms are ready when the main loop starts.
     */
    if (!CheckVCC())
    {
        for (s = 0; s < (POST_STEPS * 2U + 2U); s++)
        {
            HAL_IWDG_Refresh(&hiwdg);
            HAL_Delay(POST_STEP_MS);
        }
        return;
    }

    /* Sweep 0 → DAC_MAX */
    for (s = 0; s <= POST_STEPS; s++)
    {
        val = s * step_size;
        if (val > DAC_MAX) val = DAC_MAX;
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(POST_STEP_MS);
    }

    /* Sweep DAC_MAX → 0  (unsigned countdown idiom avoids underflow) */
    for (s = POST_STEPS + 1U; s-- > 0U; )
    {
        val = s * step_size;
        if (val > DAC_MAX) val = DAC_MAX;
        HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, val);
        HAL_IWDG_Refresh(&hiwdg);
        HAL_Delay(POST_STEP_MS);
    }

    /* Leave DAC at 0 */
    HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0);
}

/*
 * Halts the CPU for the requested period using Sleep mode (WFI).
 * SysTick continues firing every 1 ms, waking the CPU just long enough to
 * increment the HAL tick before returning to sleep — the CPU is active for
 * only a few microseconds per millisecond.
 *
 * Sleep mode — not Stop mode — is used deliberately: all peripherals stay
 * fully powered so the DAC holds its output, and the UART is ready for the
 * next poll with no re-initialisation. The IWDG also continues counting on
 * LSI and is kicked at the top of the main loop before this function is
 * called each cycle. (This is separate from the deeper Standby mode used by
 * EnterAutoStandby() below, which powers the chip down between dives.)
 */
static void SleepMs(uint32_t ms)
{
    uint32_t start = HAL_GetTick();
    while ((HAL_GetTick() - start) < ms)
        HAL_PWR_EnterSLEEPMode(PWR_MAINREGULATOR_ON, PWR_SLEEPENTRY_WFI);
}

/*
 * Called only when a successful FDO2 reading shows the unit is at/near the
 * surface (ambient pressure < SURFACE_PRESSURE_HPA) and pO2 is not
 * dangerously high (<= PPO2_SHUTDOWN_HPA). Powers the cell off, arms PA0
 * (WKUP1) as the wake source, and drops into STM32 Standby mode — the
 * lowest-power state the G051 supports. Waking behaves as a full chip reset
 * (RM0444): execution resumes at the top of main(), not back here.
 */
static void EnterAutoStandby(void)
{
    /* Cell MOSFET off (active low) before the pin configuration is lost */
    HAL_GPIO_WritePin(CELL_PWR_PORT, CELL_PWR_PIN, GPIO_PIN_SET);

    /*
     * Normal GPIO configuration does not survive Standby; the PWR
     * pull-up/down latch registers (PUCRx/PDCRx) are what hold a defined
     * pin state while the core is powered down. Force PA1 high via the
     * internal pull-up so the cell MOSFET stays off for the whole time the
     * board is asleep, regardless of any external pull network on the gate.
     */
    HAL_PWREx_EnableGPIOPullUp(PWR_GPIO_A, CELL_PWR_PIN);
    HAL_PWREx_EnablePullUpPullDownConfig();

    /* WKUP1 (PA0) is active-high by default; hardware forces an internal
     * pull-down on the pin while armed, so an unconnected/floating PA0
     * simply never wakes the board. */
    __HAL_PWR_CLEAR_FLAG(PWR_FLAG_WUF1);
    HAL_PWR_EnableWakeUpPin(PWR_WAKEUP_PIN1_HIGH);
    HAL_PWR_EnterSTANDBYMode();   /* does not return */
}

/*
 * One-time provisioning: the STM32G0 IWDG keeps counting during Standby
 * mode because it's clocked by the independent LSI oscillator, which
 * Standby does not stop. Left unfrozen, the ~4 s watchdog would reset the
 * chip roughly every 4 s while "asleep", defeating auto-shutdown entirely.
 *
 * IWDG_STDBY is a flash *option* byte, not a normal peripheral register —
 * writing it and calling HAL_FLASH_OB_Launch() triggers a full chip reset
 * immediately, so on the one boot where reprogramming is actually needed
 * this function does not return; the next boot sees the byte already
 * correct and returns immediately.
 */
static void EnsureIwdgStandbyFreeze(void)
{
    FLASH_OBProgramInitTypeDef obInit = {0};

    HAL_FLASHEx_OBGetConfig(&obInit);
    if ((obInit.USERConfig & OB_USER_IWDG_STDBY) == 0U)
        return;   /* already frozen during Standby — nothing to do */

    HAL_FLASH_Unlock();
    HAL_FLASH_OB_Unlock();

    obInit.OptionType = OPTIONBYTE_USER;
    obInit.USERType   = OB_USER_IWDG_STDBY;
    obInit.USERConfig = OB_IWDG_STDBY_FREEZE;
    if (HAL_FLASHEx_OBProgram(&obInit) != HAL_OK)
        Fatal_Init_Fault();

    HAL_FLASH_OB_Launch();   /* resets the MCU; does not return */
}

static void FDO2_FlushRx(void)
{
    uint8_t ch;
    while (HAL_UART_Receive(&huart1, &ch, 1, 5) == HAL_OK)
        ;
}

static HAL_StatusTypeDef FDO2_ReadLine(char *buf, uint16_t maxLen)
{
    uint16_t idx   = 0;
    uint8_t  ch;
    uint32_t start = HAL_GetTick();

    while (idx < maxLen - 1)
    {
        /* Unsigned subtraction handles HAL_GetTick() 32-bit rollover correctly */
        if ((HAL_GetTick() - start) >= RX_TIMEOUT_MS)
            return HAL_TIMEOUT;

        if (HAL_UART_Receive(&huart1, &ch, 1, 1) != HAL_OK)
            continue;

        if (ch == '\r')
        {
            if (idx > 0) break;
        }
        else if (ch != '\n')
        {
            buf[idx++] = (char)ch;
        }
    }
    buf[idx] = '\0';
    return (idx > 0) ? HAL_OK : HAL_TIMEOUT;
}

static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSIDiv              = RCC_HSI_DIV1;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_NONE;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
        Fatal_Init_Fault();

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_HSI;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_0) != HAL_OK)
        Fatal_Init_Fault();
}

static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOA_CLK_ENABLE();

    /*
     * Cell MOSFET (PA1) must be driven low (on) as early as possible so the
     * cell is powered before FDO2 polling begins. Pin state is written
     * before HAL_GPIO_Init() switches the pin to an output, avoiding a
     * glitch on the gate during the mode change.
     */
    HAL_GPIO_WritePin(CELL_PWR_PORT, CELL_PWR_PIN, GPIO_PIN_RESET);

    GPIO_InitStruct.Pin   = CELL_PWR_PIN;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(CELL_PWR_PORT, &GPIO_InitStruct);

    /*
     * PA0 (WKUP1) is left at its reset-default Analog state. When
     * ENABLE_AUTO_SHUTDOWN arms the wakeup pin via PWR (see
     * EnterAutoStandby()), the PWR peripheral takes control of the pin
     * electrically — including forcing an internal pull-down — regardless
     * of the GPIO peripheral's own configuration, so no GPIO_Init call for
     * WAKE_PIN is needed here.
     */

    /* PA4 — DAC output, analog mode to avoid DAC fighting GPIO driver */
    GPIO_InitStruct.Pin  = DAC_PIN;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
}

static void MX_USART1_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_USART1_CLK_ENABLE();

    GPIO_InitStruct.Pin       = FDO2_TX_PIN | FDO2_RX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;   /* pull-up handled on PCB */
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF0_USART1;
    HAL_GPIO_Init(FDO2_PORT, &GPIO_InitStruct);

    huart1.Instance          = USART1;
    huart1.Init.BaudRate     = FDO2_BAUD;
    huart1.Init.WordLength   = UART_WORDLENGTH_8B;
    huart1.Init.StopBits     = UART_STOPBITS_1;
    huart1.Init.Parity       = UART_PARITY_NONE;
    huart1.Init.Mode         = UART_MODE_TX_RX;
    huart1.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart1.Init.OverSampling = UART_OVERSAMPLING_16;
    if (HAL_UART_Init(&huart1) != HAL_OK)
        Fatal_Init_Fault();
}

static void MX_USART2_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    __HAL_RCC_USART2_CLK_ENABLE();

    GPIO_InitStruct.Pin       = DEBUG_TX_PIN;
    GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Pull      = GPIO_NOPULL;
    GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_LOW;
    GPIO_InitStruct.Alternate = GPIO_AF1_USART2;
    HAL_GPIO_Init(DEBUG_PORT, &GPIO_InitStruct);

    huart2.Instance          = USART2;
    huart2.Init.BaudRate     = DEBUG_BAUD;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.Mode         = UART_MODE_TX;   /* TX-only — nothing drives PA3/RX */
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    /*
     * Deliberately no Fatal_Init_Fault() on failure here: that would try to
     * reset via a debug path that itself just failed, and there's no LED
     * left to fall back on either. Debug output is best-effort — if it
     * doesn't come up, HAL_UART_Transmit() calls below simply return
     * HAL_ERROR and DebugLog() silently does nothing.
     */
    HAL_UART_Init(&huart2);
}

/*
 * Best-effort printf-style debug output over USART2/PA2. Silently does
 * nothing if MX_USART2_Init() failed or no debug probe/adapter is
 * connected — never affects FDO2 polling or DAC output.
 */
static void DebugLog(const char *fmt, ...)
{
    char    buf[96];
    va_list args;
    int     len;

    va_start(args, fmt);
    len = vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);

    if (len <= 0)
        return;
    if (len >= (int)sizeof(buf))
        len = (int)sizeof(buf) - 1;

    HAL_UART_Transmit(&huart2, (uint8_t *)buf, (uint16_t)len, 100);
}

static void MX_DAC_Init(void)
{
    DAC_ChannelConfTypeDef sConfig = {0};

    __HAL_RCC_DAC1_CLK_ENABLE();

    hdac.Instance = DAC1;
    if (HAL_DAC_Init(&hdac) != HAL_OK)
        Fatal_Init_Fault();

    sConfig.DAC_Trigger      = DAC_TRIGGER_NONE;
    sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
    if (HAL_DAC_ConfigChannel(&hdac, &sConfig, DAC_CHANNEL_1) != HAL_OK)
        Fatal_Init_Fault();

    if (HAL_DAC_SetValue(&hdac, DAC_CHANNEL_1, DAC_ALIGN_12B_R, 0) != HAL_OK)
        Fatal_Init_Fault();
    if (HAL_DAC_Start(&hdac, DAC_CHANNEL_1) != HAL_OK)
        Fatal_Init_Fault();
}

static void MX_IWDG_Init(void)
{
    hiwdg.Instance       = IWDG;
    hiwdg.Init.Prescaler = IWDG_PRESC;
    hiwdg.Init.Reload    = IWDG_RELOAD_VAL;
    hiwdg.Init.Window    = IWDG_WINDOW_DISABLE;
    if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
        Fatal_Init_Fault();
}

static void MX_ADC_Init(void)
{
    __HAL_RCC_ADC_CLK_ENABLE();

    hadc.Instance                   = ADC1;
    hadc.Init.ClockPrescaler        = ADC_PRESC;
    hadc.Init.Resolution            = ADC_RESOLUTION_12B;
    hadc.Init.DataAlign             = ADC_DATAALIGN_RIGHT;
    hadc.Init.ScanConvMode          = ADC_SCAN_DISABLE;
    hadc.Init.EOCSelection          = ADC_EOC_SINGLE_CONV;
    hadc.Init.LowPowerAutoWait      = DISABLE;
    hadc.Init.ContinuousConvMode    = DISABLE;
    hadc.Init.DiscontinuousConvMode = DISABLE;
    hadc.Init.ExternalTrigConv      = ADC_SOFTWARE_START;
    hadc.Init.ExternalTrigConvEdge  = ADC_EXTERNALTRIGCONVEDGE_NONE;
    hadc.Init.Overrun               = ADC_OVR_DATA_OVERWRITTEN;
    hadc.Init.SamplingTimeCommon1   = ADC_SAMP_TIME;
    hadc.Init.SamplingTimeCommon2   = ADC_SAMP_TIME;
    hadc.Init.OversamplingMode      = DISABLE;
    if (HAL_ADC_Init(&hadc) != HAL_OK)
        Fatal_Init_Fault();

    /* Factory linearity calibration — must run before first conversion */
    if (HAL_ADCEx_Calibration_Start(&hadc) != HAL_OK)
        Fatal_Init_Fault();
}

static void Fatal_Init_Fault(void)
{
    /*
     * A peripheral failed to initialise. There's no LED to indicate this
     * (the board no longer has any); cut cell power as a safe default and
     * reset so all peripherals reinitialise from scratch (automatic
     * recovery).
     */
    HAL_GPIO_WritePin(CELL_PWR_PORT, CELL_PWR_PIN, GPIO_PIN_SET);   /* MOSFET off */
    HAL_Delay(500);
    NVIC_SystemReset();
}

void SysTick_Handler(void)
{
    HAL_IncTick();
}
