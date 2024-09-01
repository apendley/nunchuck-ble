#pragma once

#define BLE_DEVICE_NAME             "NunchuckBLE"
#define BLE_DEVICE_MANUFACTURER     "squid.jpg"
#define BLE_DEVICE_MODEL            "1.0.0"
#define BLE_SERIAL_NUMBER           "Boo Berry"

// These are the settings I used for an official Nintendo controller.
#define CALIBRATION_DEAD_ZONE       (16)
#define CALIBRATION_JOY_X_OFFSET    (-5)
#define CALIBRATION_JOY_X_MIN       (53)
#define CALIBRATION_JOY_X_MAX       (195)
#define CALIBRATION_JOY_Y_OFFSET    (-5)
#define CALIBRATION_JOY_Y_MIN       (58)
#define CALIBRATION_JOY_Y_MAX       (196)

// These are the settings I used for the third party controller sold by Adafruit.
// #define CALIBRATION_DEAD_ZONE       (0)
// #define CALIBRATION_JOY_X_OFFSET    (0)
// #define CALIBRATION_JOY_X_MIN       (0)
// #define CALIBRATION_JOY_X_MAX       (255)
// #define CALIBRATION_JOY_Y_OFFSET    (0)
// #define CALIBRATION_JOY_Y_MIN       (0)
// #define CALIBRATION_JOY_Y_MAX       (255)
