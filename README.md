## Build Settings to Add
 
**C/C++ Build → Settings → MCU GCC Compiler → Preprocessor**
```
ARM_MATH_CM4
__FPU_PRESENT=1
```
 
**C/C++ Build → Settings → MCU GCC Compiler → Include paths**
```
../Drivers/CMSIS/DSP/Include
```
 
**C/C++ Build → Settings → MCU GCC Linker → Libraries (-l)**
```
arm_cortexM4lf_math
m
```
 
**C/C++ Build → Settings → MCU GCC Linker → Library search path (-L)**
```
../Drivers/CMSIS
```
 
**C/C++ Build → Settings → MCU GCC Linker → Miscellaneous**
```
-u _printf_float
```
 
**C/C++ Build → Settings → MCU/MPU Settings**
- Floating-point unit: FPv4-SP-D16
- Floating-point ABI: Hardware
---
 
## CMSIS DSP Library
 
Copy from your STM32CubeWB package:
```
STM32Cube_FW_WB_Vx.x.x/Drivers/CMSIS/DSP/Lib/GCC/libarm_cortexM4lf_math.a
```
into `Drivers/CMSIS/`
 
Copy DSP headers:
```
STM32Cube_FW_WB_Vx.x.x/Drivers/CMSIS/DSP/Include/
```
into `Drivers/CMSIS/DSP/Include/`
 
Right-click `Drivers/CMSIS/DSP/Source` in Project Explorer
→ Properties → Exclude resource from build
 
---
 
## How It Works
 
 
**Push_IMU_Data() flow:**
1. Reads LSM6DSO32 gyro + accel in one 12-byte I2C burst
2. Feeds sample into 256-point FFT buffer
3. Every 256 samples (~2.56s) FFT window completes
4. Searches bins 9-14 for 4-6Hz tremor peak
5. Logs CSV line over SWO (PB3)
6. If BLE notifications enabled, sends 32-byte packet
**BLE Packet (TremorBLEPacket_t — 32 bytes):**
 
| Bytes | Field | Format |
|-------|-------|--------|
| 1 | tremor_detected | 0=normal, 1=tremor |
| 2 | dominant_freq × 100 | uint16 e.g. 512 = 5.12Hz |
| 2 | peak_fft_mag × 10 | uint16 |
| 2 | band_energy × 10 | uint16 |
| 1 | tremor_pct | uint8 0-100% |
| 2 | gx × 100 | int16 dps |
| 2 | gy × 100 | int16 dps |
| 2 | gz × 100 | int16 dps |
| 2 | gyro_mag × 100 | uint16 dps |
| 2 | ax × 10 | int16 mg |
| 2 | ay × 10 | int16 mg |
| 2 | az × 10 | int16 mg |
| 2 | accel_mag × 10 | uint16 mg |
| 2 | jerk × 10 | uint16 mg/s |
| 4 | timestamp_ms | uint32 ms since boot |
