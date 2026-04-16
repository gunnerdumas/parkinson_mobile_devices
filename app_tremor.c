#include "main.h"
#include "lsm6dso32.h"
#include "tremor_detect.h"
#include "debug_itm.h"
#include <stdio.h>

extern I2C_HandleTypeDef hi2c1;

/* Buzzer and motor outputs (PB1, PB0) */
#define BUZZER_PORT   GPIOB
#define BUZZER_PIN    GPIO_PIN_1
#define MOTOR_PORT    GPIOB
#define MOTOR_PIN     GPIO_PIN_0

#define SAMPLE_PERIOD_MS  10     /* 104 Hz ≈ 10 ms */
#define BUZZER_BEEP_MS    200

static LSM6DSO32_Handle_t imu;
static TremorDetector_t   tremor;

void app_init(void)
{

    printf("\r\n");
    printf("==============================================\r\n");
    printf("  PARKINSONIAN TREMOR MONITOR — DEBUG BUILD\r\n");
    printf("  MCU : STM32WB50CGU5\r\n");
    printf("  IMU : LSM6DSO32  ODR=104Hz  FS=500dps\r\n");
    printf("  DBG : SWO ITM port 0 (PB3)\r\n");
    printf("==============================================\r\n\r\n");


    printf("IMU init... ");
    if (LSM6DSO32_Init(&imu, &hi2c1, LSM6DSO32_I2C_ADDR_HIGH) != HAL_OK) {
        printf("FAILED\r\n");
        printf("  Check: PB6/PB7 pull-ups, SA0 pin, power rail\r\n");
        while (1) {
            HAL_GPIO_TogglePin(BUZZER_PORT, BUZZER_PIN);
            HAL_Delay(100);
        }
    }
    printf("OK  WHO_AM_I=0x6C confirmed\r\n");


    TremorDetect_Init(&tremor);
    printf("FFT detector ready\r\n");
    printf("  Window : %d samples = %.2f s\r\n", FFT_SIZE, FFT_SIZE / SAMPLE_RATE);
    printf("  Bin    : %.4f Hz\r\n", FFT_BIN_WIDTH);
    printf("  Band   : %.1f–%.1f Hz  (bins %d–%d)\r\n\r\n",
           TREMOR_MIN_HZ, TREMOR_MAX_HZ, TREMOR_BIN_MIN, TREMOR_BIN_MAX);

    /* Column headers for data log*/
    printf("TIME_MS,"
           "GX_DPS,"          /* Raw gyro X — roll rate (dps)            */
           "GY_DPS,"          /* Raw gyro Y — pitch rate (dps)           */
           "GZ_DPS,"          /* Raw gyro Z — yaw rate (dps)             */
           "GYRO_MAG,"        /* Gyro vector magnitude (dps)             */
           "AX_MG,"           /* Raw accel X — lateral (mg)              */
           "AY_MG,"           /* Raw accel Y — longitudinal (mg)         */
           "AZ_MG,"           /* Raw accel Z — vertical/gravity (mg)     */
           "ACCEL_MAG,"       /* Accel vector magnitude (mg)             */
           "ACCEL_VERT,"      /* Vertical accel — postural context (mg)  */
           "JERK_MG_S,"       /* Jerk magnitude — accel change/dt (mg/s) */
           "DOMINANT_HZ,"     /* Peak FFT frequency in 4-6Hz band (Hz)   */
           "PEAK_FFT_MAG,"    /* FFT magnitude at dominant frequency      */
           "BAND_ENERGY,"     /* Total FFT energy in 4-6Hz band          */
           "TREMOR,"          /* 1=tremor detected, 0=normal             */
           "TREMOR_PCT\r\n"); /* % of windows with tremor since boot     */
}


void app_run(void)
{
    static uint32_t last_sample   = 0;
    static bool     prev_tremor   = false;
    static uint32_t buzzer_off_at = 0;

    uint32_t now = HAL_GetTick();


    if (buzzer_off_at && now >= buzzer_off_at) {
        HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_RESET);
        buzzer_off_at = 0;
    }


    if ((now - last_sample) >= SAMPLE_PERIOD_MS) {
        last_sample = now;
        LSM6DSO32_Data_t imu_data;
        if (LSM6DSO32_ReadAll(&imu, &imu_data) == HAL_OK) {
            TremorDetect_AddSample(&tremor,
                                   imu_data.gx, imu_data.gy, imu_data.gz,
                                   imu_data.ax, imu_data.ay, imu_data.az);

        }
    }


    if (TremorDetect_Process(&tremor)) {


        ParkinsonsData_t *d = &tremor.data;


        if (d->tremor_detected && !prev_tremor) {
            HAL_GPIO_WritePin(BUZZER_PORT, BUZZER_PIN, GPIO_PIN_SET);
            buzzer_off_at = now + BUZZER_BEEP_MS;
        }


        HAL_GPIO_WritePin(MOTOR_PORT, MOTOR_PIN,
                          d->tremor_detected ? GPIO_PIN_SET : GPIO_PIN_RESET);

        prev_tremor = d->tremor_detected;


        printf("%lu,"       /* TIME_MS      — ms since boot              */
               "%.4f,"      /* GX_DPS       — roll rate  (dps)           */
               "%.4f,"      /* GY_DPS       — pitch rate (dps)           */
               "%.4f,"      /* GZ_DPS       — yaw rate   (dps)           */
               "%.4f,"      /* GYRO_MAG     — total rotation speed (dps) */
               "%.2f,"      /* AX_MG        — lateral accel (mg)         */
               "%.2f,"      /* AY_MG        — longitudinal accel (mg)    */
               "%.2f,"      /* AZ_MG        — vertical accel (mg)        */
               "%.2f,"      /* ACCEL_MAG    — total accel magnitude (mg) */
               "%.2f,"      /* ACCEL_VERT   — gravity/postural ref (mg)  */
               "%.2f,"      /* JERK_MG_S    — jerk magnitude (mg/s)      */
               "%.4f,"      /* DOMINANT_HZ  — peak tremor frequency (Hz) */
               "%.2f,"      /* PEAK_FFT_MAG — strength of peak           */
               "%.2f,"      /* BAND_ENERGY  — total 4-6Hz band power     */
               "%d,"        /* TREMOR       — 1=active, 0=normal         */
               "%.1f\r\n",  /* TREMOR_PCT   — % time tremoring           */
               d->timestamp_ms,
               d->gx_raw,        d->gy_raw,        d->gz_raw,
               d->gyro_magnitude,
               d->ax_raw,        d->ay_raw,        d->az_raw,
               d->accel_magnitude,
               d->accel_vertical,
               d->jerk_magnitude,
               d->dominant_freq,
               d->peak_fft_mag,
               d->tremor_band_energy,
               d->tremor_detected ? 1 : 0,
               d->tremor_percent);


#ifdef DEBUG_SPECTRUM
        float spectrum[FFT_SIZE / 2];
        TremorDetect_GetSpectrum(&tremor, spectrum, FFT_SIZE / 2);
        printf("  SPECTRUM: ");
        for (uint8_t b = 0; b <= 15; b++)
            printf("[%.0fHz:%.1f]", b * FFT_BIN_WIDTH, spectrum[b]);
        printf("\r\n");
#endif
    }
}
