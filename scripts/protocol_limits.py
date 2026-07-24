"""AUTO-GENERATED from settings.yml — do not edit; run scripts/generate_settings.py"""

MOTOR_FREQ_HZ_MIN = 4
MOTOR_FREQ_HZ_MAX = 1000
MOTOR_FREQ_HZ_DEFAULT = 150
MOTOR_VDC_MV_MIN = 750
MOTOR_VDC_MV_MAX = 4200
MOTOR_VDC_MV_DEFAULT = 3100
IMU_ODR_CODE_MIN = 1
IMU_ODR_CODE_MAX = 10
IMU_ODR_CODE_DEFAULT = 7
IMU_CONTENT_MIN = 1
IMU_CONTENT_MAX = 3
IMU_CONTENT_DEFAULT = 3
IMU_ACCEL_FS_MIN = 0
IMU_ACCEL_FS_MAX = 3
IMU_ACCEL_FS_DEFAULT = 0
IMU_GYRO_FS_MIN = 0
IMU_GYRO_FS_MAX = 3
IMU_GYRO_FS_DEFAULT = 0
PREVIEW_HZ_MIN = 0
PREVIEW_HZ_MAX = 6660
PREVIEW_HZ_DEFAULT = 0
LED_ENABLED_MIN = 0
LED_ENABLED_MAX = 1
LED_ENABLED_DEFAULT = 1

LIMITS = {
    "motor_freq_hz": (4, 1000, 150),
    "motor_vdc_mv": (750, 4200, 3100),
    "imu_odr_code": (1, 10, 7),
    "imu_content": (1, 3, 3),
    "imu_accel_fs": (0, 3, 0),
    "imu_gyro_fs": (0, 3, 0),
    "preview_hz": (0, 6660, 0),
    "led_enabled": (0, 1, 1),
}
