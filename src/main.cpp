#include <bluefruit.h>
#include <WiiChuck.h>
#include <Adafruit_NeoPixel.h>
#include "Config.h"

// Uncomment define below to enable debug logging in this file.
// #define LOGGER Serial
#include "Logger.h"

static const char* revision = "1.0.0";

////////////////////////////
// Drivers/variables
////////////////////////////
Adafruit_NeoPixel pixel(1, 8, NEO_GRB + NEO_KHZ800);

Accessory nunchuck;
hid_gamepad_report_t lastGamepadReport = {0};

BLEDis bleDis;
BLEHidGamepad bleGamepad;

bool isPairing = false;

////////////////////////////
// Forward declarations
////////////////////////////
bool initNunchuck();
void initBLE();
void startAdvertising(void);

bool pairPasskeyCallback(uint16_t connHandle, uint8_t const passkey[6], bool matchRequest);
void connectCallback(uint16_t connHandle);
void pairCompleteCallback(uint16_t connHandle, uint8_t authStatus);
void securedCallback(uint16_t connHandle);
void disconnectCallback(uint16_t connHandle, uint8_t reason);

////////////////////////////
// Setup
////////////////////////////
void setup() {
    pixel.begin();
    pixel.fill(0);
    pixel.show();

    Serial.begin(115200);
    // while(!Serial) { delay(10); }    

    initBLE();

    // If this fails, plug in nunchuck and power cycle to try again.
    // Also, make sure it is indeed a nunchuck plugged in.
    // Other controllers aren't (yet) supported.
    if (!initNunchuck()) {
        pixel.fill(0x100000);
        pixel.show();

        while(true) {
            delay(10);
        }
    }

    // If both c and z buttons are pressed, enter pairing mode.
    if (nunchuck.readData()) {
        if (nunchuck.getButtonC() && nunchuck.getButtonZ()) {
            LOGLN("Pairing mode...");
            Bluefruit.Periph.clearBonds();            
            isPairing = true;
        }
    }

    // Set up and start advertising
    startAdvertising();
}

////////////////////////////
// Loop
////////////////////////////
void loop() {
    bool didReadNunchuck = nunchuck.readData();

    if (isPairing) {
        pixel.fill(0x001000);
    } 
    else if (didReadNunchuck) {
        pixel.fill(0);
    } 
    else {
        // If not intermittent (i.e. if LED stays red), check the nunchuck connection, and power cycle.
        pixel.fill(0x100000);
    }    

    pixel.show();    

    // Not much to do right now if we aren't connected or couldn't read from the nunchuck.
    if (!(Bluefruit.connected() && didReadNunchuck)) {
        delay(5);            
        return;
    }

    uint8_t xRaw = nunchuck.getJoyX();
    uint8_t yRaw = nunchuck.getJoyY();
    int16_t xJoy = map(xRaw + CALIBRATION_JOY_X_OFFSET, CALIBRATION_JOY_X_MIN, CALIBRATION_JOY_X_MAX, -127, 127);
    int16_t yJoy = map(yRaw + CALIBRATION_JOY_Y_OFFSET, CALIBRATION_JOY_Y_MIN, CALIBRATION_JOY_Y_MAX, -127, 127);
    xJoy = min(127, max(-127, xJoy));
    yJoy = min(127, max(-127, yJoy));

    if (abs(xJoy) < CALIBRATION_DEAD_ZONE) {
        xJoy = 0;
    }

    if (abs(yJoy) < CALIBRATION_DEAD_ZONE) {
        yJoy = 0;
    }    

    // LOGFMT("xRaw: %d, yRaw: %d\n", xRaw, yRaw);
    // LOGFMT("xJoy: %d, yJoy: %d\n\n", xJoy, yJoy);

    int8_t ax = map(nunchuck.getAccelX(), 0, 1024, -127, 127);
    int8_t ay = map(nunchuck.getAccelY(), 0, 1024, -127, 127);
    int8_t az = map(nunchuck.getAccelZ(), 0, 1024, -127, 127);
    ax = min(127, max(-127, ax));
    ay = min(127, max(-127, ay));    
    az = min(127, max(-127, az));

    // LOGFMT("axRaw: %d, ayRaw: %d, azRaw: %d\n", nunchuck.getAccelX(), nunchuck.getAccelY(), nunchuck.getAccelZ());
    // LOGFMT("ax: %d, ay: %d, az: %d\n", ax, ay, az);

    hid_gamepad_report_t report = {0};
    report.x = xJoy;
    report.y = yJoy;
    report.rx = ax;
    report.ry = ay;
    report.rz = az;

    if (nunchuck.getButtonZ()) {
        report.buttons |= GAMEPAD_BUTTON_Z;
    }

    if (nunchuck.getButtonC()) {
        report.buttons |= GAMEPAD_BUTTON_C;
    }

    if (Bluefruit.connected()) {
        if (memcmp(&lastGamepadReport, &report, sizeof(hid_gamepad_report_t))) {
            bleGamepad.report(&report);
            memcpy(&lastGamepadReport, &report, sizeof(hid_gamepad_report_t));
            delay(1);
        }  
    }
}

////////////////////////////
// Function definitions
////////////////////////////
bool initNunchuck() {
    nunchuck.begin();

    // Currently only support the nunchuck.
    if (nunchuck.type != NUNCHUCK) {
        return false;
    }

    // Attempt to read; if we fail, return false.
    if (!nunchuck.readData()) {
        return false;
    }

    return true;
}

void initBLE() {
    Bluefruit.begin();
    Bluefruit.setTxPower(4);
    Bluefruit.setName(BLE_DEVICE_NAME);

    // Configure and Start Device Information Service
    bleDis.setManufacturer(BLE_DEVICE_MANUFACTURER);
    bleDis.setModel(BLE_DEVICE_MODEL);
    bleDis.setFirmwareRev(revision);
    bleDis.setSoftwareRev(revision);
    bleDis.setSerialNum(BLE_SERIAL_NUMBER);
    bleDis.begin();

    // Configure gamepad
    // display = true, yes/no = true, keyboard = false.
    // We're not telling the truth here. We don't have a screen or yes/not buttons.
    // When the pairing callback is exectued,  we simply allow pairing if we're 
    // in pairing mode, otherwise we don't.
    Bluefruit.Security.setIOCaps(true, true, false);
    Bluefruit.Security.setMITM(true);
    Bluefruit.Security.setPairPasskeyCallback(pairPasskeyCallback);
    Bluefruit.Security.setPairCompleteCallback(pairCompleteCallback);
    Bluefruit.Security.setSecuredCallback(securedCallback);
    Bluefruit.Periph.setConnectCallback(connectCallback);
    Bluefruit.Periph.setDisconnectCallback(disconnectCallback);    

    bleGamepad.setPermission(SECMODE_ENC_WITH_MITM, SECMODE_ENC_WITH_MITM);
    bleGamepad.begin();
}

void startAdvertising(void) {
    // Advertising packet
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addTxPower();
    Bluefruit.Advertising.addAppearance(BLE_APPEARANCE_HID_GAMEPAD);
    
    // Include BLE HID service
    Bluefruit.Advertising.addService(bleGamepad);
    
    // Add the name to the scan response if we can't add it to the main packet.
    if (!Bluefruit.Advertising.addName()) {
        Bluefruit.ScanResponse.addName();
    }
    
    Bluefruit.Advertising.restartOnDisconnect(true);

    // in unit of 0.625 ms
    Bluefruit.Advertising.setInterval(32, 244);

    // number of seconds in fast mode
    Bluefruit.Advertising.setFastTimeout(30);

    // 0 = Don't stop advertising after n seconds
    Bluefruit.Advertising.start(0);
}

void connectCallback(uint16_t connHandle) {
    LOGLN("Connected");
}

bool pairPasskeyCallback(uint16_t connHandle, uint8_t const passkey[6], bool matchRequest) {
    LOGFMT("pairPasskeyCallback, isPairing: %d\n", isPairing);
    return isPairing;
}

void pairCompleteCallback(uint16_t connHandle, uint8_t authStatus) {
    // Handy reference for authStatus values.
    // #define BLE_GAP_SEC_STATUS_SUCCESS                0x00  /**< Procedure completed with success. */
    // #define BLE_GAP_SEC_STATUS_TIMEOUT                0x01  /**< Procedure timed out. */
    // #define BLE_GAP_SEC_STATUS_PDU_INVALID            0x02  /**< Invalid PDU received. */
    // #define BLE_GAP_SEC_STATUS_RFU_RANGE1_BEGIN       0x03  /**< Reserved for Future Use range #1 begin. */
    // #define BLE_GAP_SEC_STATUS_RFU_RANGE1_END         0x80  /**< Reserved for Future Use range #1 end. */
    // #define BLE_GAP_SEC_STATUS_PASSKEY_ENTRY_FAILED   0x81  /**< Passkey entry failed (user canceled or other). */
    // #define BLE_GAP_SEC_STATUS_OOB_NOT_AVAILABLE      0x82  /**< Out of Band Key not available. */
    // #define BLE_GAP_SEC_STATUS_AUTH_REQ               0x83  /**< Authentication requirements not met. */
    // #define BLE_GAP_SEC_STATUS_CONFIRM_VALUE          0x84  /**< Confirm value failed. */
    // #define BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP       0x85  /**< Pairing not supported.  */
    // #define BLE_GAP_SEC_STATUS_ENC_KEY_SIZE           0x86  /**< Encryption key size. */
    // #define BLE_GAP_SEC_STATUS_SMP_CMD_UNSUPPORTED    0x87  /**< Unsupported SMP command. */
    // #define BLE_GAP_SEC_STATUS_UNSPECIFIED            0x88  /**< Unspecified reason. */
    // #define BLE_GAP_SEC_STATUS_REPEATED_ATTEMPTS      0x89  /**< Too little time elapsed since last attempt. */
    // #define BLE_GAP_SEC_STATUS_INVALID_PARAMS         0x8A  /**< Invalid parameters. */
    // #define BLE_GAP_SEC_STATUS_DHKEY_FAILURE          0x8B  /**< DHKey check failure. */
    // #define BLE_GAP_SEC_STATUS_NUM_COMP_FAILURE       0x8C  /**< Numeric Comparison failure. */
    // #define BLE_GAP_SEC_STATUS_BR_EDR_IN_PROG         0x8D  /**< BR/EDR pairing in progress. */
    // #define BLE_GAP_SEC_STATUS_X_TRANS_KEY_DISALLOWED 0x8E  /**< BR/EDR Link Key cannot be used for LE keys. */
    // #define BLE_GAP_SEC_STATUS_RFU_RANGE2_BEGIN       0x8F  /**< Reserved for Future Use range #2 begin. */
    // #define BLE_GAP_SEC_STATUS_RFU_RANGE2_END         0xFF  /**< Reserved for Future Use range #2 end. */
    LOGFMT("Pairing status: 0x%X\n", authStatus);    
}

void securedCallback(uint16_t connHandle) {
    LOGLN("Connection secured");
    isPairing = false;

    hid_gamepad_report_t report = {0};
    memcpy(&lastGamepadReport, &report, sizeof(hid_gamepad_report_t));
}

void disconnectCallback(uint16_t connHandle, uint8_t reason) {
    // Handy reference for reason values.
    // #define BLE_HCI_STATUS_CODE_SUCCESS                                0x00
    // #define BLE_HCI_STATUS_CODE_UNKNOWN_BTLE_COMMAND                   0x01
    // #define BLE_HCI_STATUS_CODE_UNKNOWN_CONNECTION_IDENTIFIER          0x02
    // #define BLE_HCI_AUTHENTICATION_FAILURE                             0x05
    // #define BLE_HCI_STATUS_CODE_PIN_OR_KEY_MISSING                     0x06
    // #define BLE_HCI_MEMORY_CAPACITY_EXCEEDED                           0x07
    // #define BLE_HCI_CONNECTION_TIMEOUT                                 0x08
    // #define BLE_HCI_STATUS_CODE_COMMAND_DISALLOWED                     0x0C
    // #define BLE_HCI_STATUS_CODE_INVALID_BTLE_COMMAND_PARAMETERS        0x12
    // #define BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION                  0x13
    // #define BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_LOW_RESOURCES        0x14
    // #define BLE_HCI_REMOTE_DEV_TERMINATION_DUE_TO_POWER_OFF            0x15
    // #define BLE_HCI_LOCAL_HOST_TERMINATED_CONNECTION                   0x16
    // #define BLE_HCI_UNSUPPORTED_REMOTE_FEATURE                         0x1A
    // #define BLE_HCI_STATUS_CODE_INVALID_LMP_PARAMETERS                 0x1E
    // #define BLE_HCI_STATUS_CODE_UNSPECIFIED_ERROR                      0x1F
    // #define BLE_HCI_STATUS_CODE_LMP_RESPONSE_TIMEOUT                   0x22
    // #define BLE_HCI_STATUS_CODE_LMP_ERROR_TRANSACTION_COLLISION        0x23
    // #define BLE_HCI_STATUS_CODE_LMP_PDU_NOT_ALLOWED                    0x24
    // #define BLE_HCI_INSTANT_PASSED                                     0x28
    // #define BLE_HCI_PAIRING_WITH_UNIT_KEY_UNSUPPORTED                  0x29
    // #define BLE_HCI_DIFFERENT_TRANSACTION_COLLISION                    0x2A
    // #define BLE_HCI_PARAMETER_OUT_OF_MANDATORY_RANGE                   0x30
    // #define BLE_HCI_CONTROLLER_BUSY                                    0x3A
    // #define BLE_HCI_CONN_INTERVAL_UNACCEPTABLE                         0x3B
    // #define BLE_HCI_DIRECTED_ADVERTISER_TIMEOUT                        0x3C
    // #define BLE_HCI_CONN_TERMINATED_DUE_TO_MIC_FAILURE                 0x3D
    // #define BLE_HCI_CONN_FAILED_TO_BE_ESTABLISHED                      0x3E    
    LOGFMT("Disconnected, reason: 0x%X\n", reason);
}
