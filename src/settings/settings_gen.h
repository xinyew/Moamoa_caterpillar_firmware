/* AUTO-GENERATED from settings.yml — do not edit; run scripts/generate_settings.py */
/* settings.yml sha1: 5fc797296a1700336b302b0406dde089263689e1 */
#ifndef SETTINGS_GEN_H
#define SETTINGS_GEN_H

#include <stdint.h>

#define SETTING_MOTOR_FREQ_HZ        1
#define SETTING_MOTOR_VDC_MV         2
#define SETTING_IMU_ODR_CODE         3
#define SETTING_IMU_CONTENT          4
#define SETTING_IMU_ACCEL_FS         5
#define SETTING_IMU_GYRO_FS          6
#define SETTING_PREVIEW_HZ           7
#define SETTING_LED_ENABLED          8
#define SETTING_ROBOT_ID             9

#define SETTINGS_COUNT 9
#define SETTINGS_MAX_ID 9

struct setting_meta {
    uint8_t  id;
    const char *name;
    uint16_t def;
    uint16_t min;
    uint16_t max;
};

#define SETTINGS_TABLE { \
    { 1, "motor_freq_hz", 150, 4, 1000 }, \
    { 2, "motor_vdc_mv", 3100, 750, 4200 }, \
    { 3, "imu_odr_code", 7, 1, 10 }, \
    { 4, "imu_content", 3, 1, 3 }, \
    { 5, "imu_accel_fs", 0, 0, 3 }, \
    { 6, "imu_gyro_fs", 0, 0, 3 }, \
    { 7, "preview_hz", 0, 0, 6660 }, \
    { 8, "led_enabled", 1, 0, 1 }, \
    { 9, "robot_id", 0, 0, 20 }, \
}

#endif /* SETTINGS_GEN_H */
