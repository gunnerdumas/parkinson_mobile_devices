# Parkinson's Disease Tremor Detection
## Branch: Tremor-Detect


---

### Files Added
| File | Description |
|------|-------------|
| Core/Inc/lsm6dso32.h | IMU driver header |
| Core/Src/lsm6dso32.c | IMU driver - gyro + accel via I2C |
| Core/Inc/tremor_detect.h | FFT algorithm + ParkinsonsData_t struct |
| Core/Src/tremor_detect.c | CMSIS-DSP 256-point FFT tremor detection |
| Core/Inc/debug_itm.h | SWO printf header |
| Core/Src/debug_itm.c | SWO printf via ITM port 0 |
| STM32_WPAN/App/custom_app.c | BLE + IMU integration, Push_IMU_Data() |
| STM32_WPAN/App/custom_stm.c | BLE characteristic config, 32-byte packet |
| SWV_export/ | Recorded test data CSV files |

---

### Algorithm Overview

**Sampling:** LSM6DSO32 sampled at 104Hz via I2C every 10ms

**FFT Pipeline:**
1. DC removal - subtract mean from buffer
2. Hann window - reduce spectral leakage
3. 3-tap pre-filter - smooth high frequency noise
4. 256-point arm_rfft_fast_f32 - frequency domain conversion
5. arm_cmplx_mag_f32 - compute magnitude spectrum
6. Peak search in 4-6 Hz band - tremor frequency detection
7. Global peak search - dominant frequency across full 0-52 Hz spectrum
8. Hysteresis - 2 confirm windows ON, 3 clear windows OFF

**Velocity Gate:**
Suppresses false positives during voluntary movement.
Large gyro magnitude greater than 35 dps blocks tremor detection.
Parkinsonian resting tremor produces low gyro magnitude (2-15 dps).

---

### BLE Packet Format
Notifications sent every 100ms on DATA_IMU characteristic
UUID: 00000001-8E22-4541-9D4C-21EDAE82ED19

| Field | Type | Scale | Description |
|-------|------|-------|-------------|
| tremor_detected | uint8 | - | 0=none, 1=tremor |
| dominant_freq_x100 | uint16 | divide by 100 = Hz | Peak in 4-6Hz band |
| peak_fft_mag_x10 | uint16 | divide by 10 | FFT magnitude |
| band_energy_x10 | uint16 | divide by 10 | Total 4-6Hz energy |
| tremor_pct | uint8 | % | Session tremor % |
| gx_x100 | int16 | divide by 100 = dps | Gyro X |
| gy_x100 | int16 | divide by 100 = dps | Gyro Y |
| gz_x100 | int16 | divide by 100 = dps | Gyro Z |
| gyro_mag_x100 | uint16 | divide by 100 = dps | Gyro magnitude |
| ax_x10 | int16 | divide by 10 = mg | Accel X |
| ay_x10 | int16 | divide by 10 = mg | Accel Y |
| az_x10 | int16 | divide by 10 = mg | Accel Z |
| accel_mag_x10 | uint16 | divide by 10 = mg | Accel magnitude |
| jerk_x10 | uint16 | divide by 10 = mg/s | Jerk magnitude |
| global_freq_x100 | uint16 | divide by 100 = Hz | Global peak frequency |
| global_mag_x10 | uint16 | divide by 10 | Global peak magnitude |
| timestamp_ms | uint32 | ms | HAL_GetTick() |

Total packet size: 32 bytes

---

### CubeIDE Build Settings

**Preprocessor** - C/C++ Build > Settings > MCU GCC Compiler > Preprocessor
```
ARM_MATH_CM4
__FPU_PRESENT=1
```
**Include Paths** - C/C++ Build > Settings > MCU GCC Compiler > Include paths
```
../Drivers/CMSIS/DSP/Include
```
**Libraries (-l)** - C/C++ Build > Settings > MCU GCC Linker > Libraries
```
arm_cortexM4lf_math
m
```
**Library Search Path (-L)** - C/C++ Build > Settings > MCU GCC Linker > Libraries
```
../Drivers/CMSIS
```
**Linker Misc** - C/C++ Build > Settings > MCU GCC Linker > Miscellaneous
```
-u _printf_float
```

**FPU** - C/C++ Build > Settings > MCU/MPU Settings
- Floating-point unit: FPv4-SP-D16
- Floating-point ABI: Hardware

**Exclude from build** - right-click Drivers/CMSIS/DSP/Source > Properties > Exclude resource from build

---
