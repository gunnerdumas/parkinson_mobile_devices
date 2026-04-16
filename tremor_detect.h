#ifndef TREMOR_DETECT_H
#define TREMOR_DETECT_H

#include "stm32wbxx_hal.h"
#include "arm_math.h"
#include <stdint.h>
#include <stdbool.h>

/*
 * Sample rate : 104 Hz
 * Tremor band : 4–6 Hz
 */

#define SAMPLE_RATE           104.0f
#define FFT_SIZE              256
#define FFT_BIN_WIDTH         (SAMPLE_RATE / FFT_SIZE)

#define TREMOR_MIN_HZ         4.0f
#define TREMOR_MAX_HZ         6.0f
#define TREMOR_BIN_MIN        ((uint16_t)(TREMOR_MIN_HZ / FFT_BIN_WIDTH))
#define TREMOR_BIN_MAX        ((uint16_t)(TREMOR_MAX_HZ / FFT_BIN_WIDTH))
#define TREMOR_FFT_THRESHOLD  50.0f

#define CONFIRM_WINDOWS       2
#define CLEAR_WINDOWS         3
#define PREFILTER_SIZE        3


typedef struct {

    float gx_raw;           /* X-axis angular velocity (dps) */
    float gy_raw;           /* Y-axis angular velocity (dps) */
    float gz_raw;           /* Z-axis angular velocity (dps) */


    float gyro_magnitude;


    float ax_raw;           /* X-axis linear acceleration (mg) */
    float ay_raw;           /* Y-axis linear acceleration (mg) */
    float az_raw;           /* Z-axis linear acceleration (mg) */


    float accel_magnitude;  /* mg */


    float accel_vertical;   /* mg — same as az_raw */


    float jerk_magnitude;   /* mg/s */

    /* Computed once per window (128 samples = ~1.23 s). */
    float dominant_freq;    /* Hz  — peak bin in 4–6 Hz band  */
    float peak_fft_mag;     /* FFT magnitude at dominant_freq  */
    float bin_width;        /* Hz/bin = 0.8125 Hz */

    /* Sum of FFT magnitudes across all bins in 4–6 Hz range. */
    float tremor_band_energy; /* sum of fft_mag[bin_min..bin_max] */

    bool  tremor_detected;


    uint32_t total_windows;
    uint32_t tremor_windows;
    float    tremor_percent;


    uint32_t timestamp_ms;

} ParkinsonsData_t;


typedef struct {
    float    raw_buf[FFT_SIZE];
    uint16_t sample_count;
    float    prefilter_buf[PREFILTER_SIZE];
    uint8_t  prefilter_idx;

    arm_rfft_fast_instance_f32 fft_inst;
    float    windowed[FFT_SIZE];
    float    fft_out[FFT_SIZE];
    float    fft_mag[FFT_SIZE / 2];
    float    hann[FFT_SIZE];

    uint8_t  confirm_count;
    uint8_t  clear_count;

    ParkinsonsData_t data;   /* live data — read after Process() returns true */
} TremorDetector_t;

/* ---- API ---- */
void  TremorDetect_Init      (TremorDetector_t *td);

/* Pass both gyro and accel from each LSM6DSO32_ReadAll() call.
 * accel units: mg. gyro units: dps.                           */
void  TremorDetect_AddSample (TremorDetector_t *td,
                               float gx, float gy, float gz,
                               float ax, float ay, float az);

bool  TremorDetect_Process   (TremorDetector_t *td);  /* true = new window ready */


void  TremorDetect_GetSpectrum(TremorDetector_t *td, float *out, uint16_t len);

#endif
