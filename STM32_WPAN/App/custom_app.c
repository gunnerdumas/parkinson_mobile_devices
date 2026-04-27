/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    App/custom_app.c
  * @author  MCD Application Team
  * @brief   Custom Example Application (Server)
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "app_common.h"
#include "dbg_trace.h"
#include "ble.h"
#include "custom_app.h"
#include "custom_stm.h"
#include "stm32_seq.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "lsm6dso32.h"
#include "tremor_detect.h"
#include "debug_itm.h"
#include <stdio.h>
#include <string.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
typedef struct
{
  /* parkinson_data */
  uint8_t               Data_imu_Notification_Status;
  /* USER CODE BEGIN CUSTOM_APP_Context_t */

  /* USER CODE END CUSTOM_APP_Context_t */

  uint16_t              ConnectionHandle;
} Custom_App_Context_t;

/* USER CODE BEGIN PTD */
typedef struct __attribute__((packed)) {
    uint8_t  tremor_detected;
    uint16_t dominant_freq_x100;
    uint16_t peak_fft_mag_x10;
    uint16_t band_energy_x10;
    uint8_t  tremor_pct;
    int16_t  gx_x100;
    int16_t  gy_x100;
    int16_t  gz_x100;
    uint16_t gyro_mag_x100;
    int16_t  ax_x10;
    int16_t  ay_x10;
    int16_t  az_x10;
    uint16_t accel_mag_x10;
    uint16_t jerk_x10;
    uint32_t timestamp_ms;
    uint16_t global_freq_x100;
    uint16_t global_mag_x10;
} TremorBLEPacket_t;
/* USER CODE END PTD */

/* Private defines ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macros -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/**
 * START of Section BLE_APP_CONTEXT
 */

static Custom_App_Context_t Custom_App_Context;

/**
 * END of Section BLE_APP_CONTEXT
 */

uint8_t UpdateCharData[512];
uint8_t NotifyCharData[512];
uint16_t Connection_Handle;
/* USER CODE BEGIN PV */
uint8_t notifyStatus = 0;
uint8_t timerStatus = 0;

static LSM6DSO32_Handle_t imu;
static TremorDetector_t   tremor;
static bool               imu_ready = false;

extern I2C_HandleTypeDef hi2c1;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* parkinson_data */
static void Custom_Data_imu_Update_Char(void);
static void Custom_Data_imu_Send_Notification(void);

/* USER CODE BEGIN PFP */

void Push_IMU_Data(void)
{
    if (!imu_ready) return;

    static uint8_t raw_tick = 0;

    // Sample IMU
    LSM6DSO32_Data_t imu_data;
    if (LSM6DSO32_ReadAll(&imu, &imu_data) != HAL_OK) return;

    TremorDetect_AddSample(&tremor,
                           imu_data.gx, imu_data.gy, imu_data.gz,
                           imu_data.ax, imu_data.ay, imu_data.az);

    // Only process when FFT window is full
    TremorDetect_Process(&tremor);

    raw_tick++;
    if (raw_tick < 10) return;
    raw_tick = 0;

    ParkinsonsData_t *d = &tremor.data;

    // SWO CSV log
    printf("%lu,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%.4f,%.2f,%d,%.1f\r\n",
           HAL_GetTick(),
           d->gx_raw, d->gy_raw, d->gz_raw, d->gyro_magnitude,
           d->ax_raw, d->ay_raw, d->az_raw, d->accel_magnitude,
           d->accel_vertical, d->jerk_magnitude,
           d->dominant_freq, d->peak_fft_mag, d->tremor_band_energy,
           d->global_dominant_freq, d->global_peak_fft_mag,
           d->tremor_detected ? 1 : 0,
           d->tremor_percent);

    // Pack BLE payload
    TremorBLEPacket_t pkt;
    pkt.tremor_detected    = d->tremor_detected ? 1 : 0;
    pkt.dominant_freq_x100 = (uint16_t)(d->dominant_freq      * 100.0f);
    pkt.peak_fft_mag_x10   = (uint16_t)(d->peak_fft_mag       * 10.0f);
    pkt.band_energy_x10    = (uint16_t)(d->tremor_band_energy  * 10.0f);
    pkt.tremor_pct         = (uint8_t)  d->tremor_percent;
    pkt.gx_x100            = (int16_t) (d->gx_raw             * 100.0f);
    pkt.gy_x100            = (int16_t) (d->gy_raw             * 100.0f);
    pkt.gz_x100            = (int16_t) (d->gz_raw             * 100.0f);
    pkt.gyro_mag_x100      = (uint16_t)(d->gyro_magnitude      * 100.0f);
    pkt.ax_x10             = (int16_t) (d->ax_raw             * 10.0f);
    pkt.ay_x10             = (int16_t) (d->ay_raw             * 10.0f);
    pkt.az_x10             = (int16_t) (d->az_raw             * 10.0f);
    pkt.accel_mag_x10      = (uint16_t)(d->accel_magnitude     * 10.0f);
    pkt.jerk_x10           = (uint16_t)(d->jerk_magnitude      * 10.0f);
    pkt.timestamp_ms       = HAL_GetTick();
    pkt.global_freq_x100   = (uint16_t)(d->global_dominant_freq * 100.0f);
    pkt.global_mag_x10     = (uint16_t)(d->global_peak_fft_mag  * 10.0f);


    memcpy(UpdateCharData, &pkt, sizeof(TremorBLEPacket_t));
    Custom_Data_imu_Update_Char();
}

/* USER CODE END PFP */

/* Functions Definition ------------------------------------------------------*/
void Custom_STM_App_Notification(Custom_STM_App_Notification_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_1 */

  /* USER CODE END CUSTOM_STM_App_Notification_1 */
  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* USER CODE END CUSTOM_STM_App_Notification_Custom_Evt_Opcode */

    /* parkinson_data */
    case CUSTOM_STM_DATA_IMU_NOTIFY_ENABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DATA_IMU_NOTIFY_ENABLED_EVT */
    	notifyStatus=1;
      /* USER CODE END CUSTOM_STM_DATA_IMU_NOTIFY_ENABLED_EVT */
      break;

    case CUSTOM_STM_DATA_IMU_NOTIFY_DISABLED_EVT:
      /* USER CODE BEGIN CUSTOM_STM_DATA_IMU_NOTIFY_DISABLED_EVT */
    	notifyStatus=0;
      /* USER CODE END CUSTOM_STM_DATA_IMU_NOTIFY_DISABLED_EVT */
      break;

    case CUSTOM_STM_NOTIFICATION_COMPLETE_EVT:
      /* USER CODE BEGIN CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */

      /* USER CODE END CUSTOM_STM_NOTIFICATION_COMPLETE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_STM_App_Notification_default */

      /* USER CODE END CUSTOM_STM_App_Notification_default */
      break;
  }
  /* USER CODE BEGIN CUSTOM_STM_App_Notification_2 */

  /* USER CODE END CUSTOM_STM_App_Notification_2 */
  return;
}

void Custom_APP_Notification(Custom_App_ConnHandle_Not_evt_t *pNotification)
{
  /* USER CODE BEGIN CUSTOM_APP_Notification_1 */

  /* USER CODE END CUSTOM_APP_Notification_1 */

  switch (pNotification->Custom_Evt_Opcode)
  {
    /* USER CODE BEGIN CUSTOM_APP_Notification_Custom_Evt_Opcode */

    /* USER CODE END P2PS_CUSTOM_Notification_Custom_Evt_Opcode */
    case CUSTOM_CONN_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_CONN_HANDLE_EVT */

      /* USER CODE END CUSTOM_CONN_HANDLE_EVT */
      break;

    case CUSTOM_DISCON_HANDLE_EVT :
      /* USER CODE BEGIN CUSTOM_DISCON_HANDLE_EVT */

      /* USER CODE END CUSTOM_DISCON_HANDLE_EVT */
      break;

    default:
      /* USER CODE BEGIN CUSTOM_APP_Notification_default */

      /* USER CODE END CUSTOM_APP_Notification_default */
      break;
  }

  /* USER CODE BEGIN CUSTOM_APP_Notification_2 */

  /* USER CODE END CUSTOM_APP_Notification_2 */

  return;
}

void Custom_APP_Init(void)
{
  /* USER CODE BEGIN CUSTOM_APP_Init */
  printf("\r\n=== PARKINSON TREMOR MONITOR ===\r\n");
  printf("IMU init... ");
  if (LSM6DSO32_Init(&imu, &hi2c1, LSM6DSO32_I2C_ADDR_HIGH) != HAL_OK) {
      printf("FAILED\r\n");
      imu_ready = false;
  } else {
      printf("OK\r\n");
      TremorDetect_Init(&tremor);
      imu_ready = true;
      printf("TIME_MS,GX_DPS,GY_DPS,GZ_DPS,GYRO_MAG,AX_MG,AY_MG,AZ_MG,"
             "ACCEL_MAG,ACCEL_VERT,JERK_MG_S,DOMINANT_HZ,PEAK_FFT_MAG,"
             "BAND_ENERGY,GLOBAL_HZ,GLOBAL_MAG,TREMOR,TREMOR_PCT\r\n");
  }
  /* USER CODE END CUSTOM_APP_Init */
  return;
}

/* USER CODE BEGIN FD */

/* USER CODE END FD */

/*************************************************************
 *
 * LOCAL FUNCTIONS
 *
 *************************************************************/

/* parkinson_data */
__USED void Custom_Data_imu_Update_Char(void) /* Property Read */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Data_imu_UC_1*/
  	updateflag=notifyStatus;
    uint8_t testData=20;
  /* USER CODE END Data_imu_UC_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DATA_IMU, (uint8_t *)UpdateCharData);
  }

  /* USER CODE BEGIN Data_imu_UC_Last*/

  /* USER CODE END Data_imu_UC_Last*/
  return;
}

void Custom_Data_imu_Send_Notification(void) /* Property Notification */
{
  uint8_t updateflag = 0;

  /* USER CODE BEGIN Data_imu_NS_1*/
  updateflag=notifyStatus;
  uint8_t testData=20;
  /* USER CODE END Data_imu_NS_1*/

  if (updateflag != 0)
  {
    Custom_STM_App_Update_Char(CUSTOM_STM_DATA_IMU, (uint8_t *)NotifyCharData);
  }

  /* USER CODE BEGIN Data_imu_NS_Last*/

  /* USER CODE END Data_imu_NS_Last*/

  return;
}

/* USER CODE BEGIN FD_LOCAL_FUNCTIONS*/

/* USER CODE END FD_LOCAL_FUNCTIONS*/
