
// 8 x 8 Array TOF Sensor I2C Mode
// Part: DFRobot SEN0628

#include "DFRobot_MatrixLidar.h"
#include <Wire.h>

DFRobot_MatrixLidar_I2C tof(0x33, &Wire1);

uint16_t buf[64];

void tof_init() {

    Serial.begin(115200);     // USB debugging
    Serial2.begin(115200);    // Bluetooth

    Wire1.begin();

    // if (!bluetoothHandshake()) {
    //     while(1) {
    //         Serial.println("No Bluetooth connection");
    //         delay(1000);
    //     }
    // }

    while (tof.begin() != 0) {
        Serial.println("begin error");
        delay(1000);
    }

    Serial.println("begin success");

    while (tof.setRangingMode(eMatrix_8X8) != 0) {
        Serial.println("init error");
        delay(1000);
    }

}

void tof_exe() {

    tof.getAllData(buf);

    // Send 8x8 matrix as CSV over Bluetooth
    for (int i = 0; i < 64; i++) {

        Serial2.print(buf[i]);

        if (i < 63)
            Serial2.print(",");
    }

    Serial2.println();

    delay(50);
}

void tof_visualize() {
    // use pyserial

}

