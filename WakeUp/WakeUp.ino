#include <Wire.h>
#include <hardware/flash.h>

//#include "consensus.h"

int ID;
const int NUMBER_OF_RPI = 3;
int STATE = 1;  // Wake Up State
bool HUB_FLAG = false;
/*
 * 0 - Idle
 * 1 - Start/Restart
 * 2 - Calibrate
 */

// I2C Message Structure
struct i2c_msg {
    uint8_t node;      // source addr
    uint32_t t;        // sample time in ms
    uint16_t value_i;  // sample value (integer)
    float value_f;     // sample value (float)
    char cmd;          // command specification
    char sub_cmd;      // subcommand specification
    i2c_msg(uint8_t addr = 0, char c = '\0', char sc = '\0', uint16_t i = 0,
            float f = 0.0f)
        : t{millis()},
          node{addr},
          cmd{c},
          sub_cmd{sc},
          value_i{i},
          value_f{f} {}
};

// Basic I2C Communication Variables
const int msg_size = sizeof(i2c_msg);
const int frame_size = msg_size + 1;  // sometimes the byte is lost in the
                                      // comms, so we send and extra dummy one
const int input_fifo_size =
    32;  // reserves 32 slots - increase if more are needed
volatile i2c_msg input_fifo[input_fifo_size];
volatile bool input_slot_full[input_fifo_size]{false};
volatile int n_frame_errors_underrun = 0;
volatile int n_frame_errors_overrun = 0;
volatile int n_overflows = 0;
volatile int error_frame_size;
int i2c_error_code = 0;
int n_i2c_errors = 0;
uint8_t this_pico_id[8];
uint8_t i2c_address;

// More I2C and System variables
float gains[3]{1, 1, 1};
uint8_t i2c_addresses[NUMBER_OF_RPI] = {0};

void setup() {
    delay(10000);
    randomSeed(analogRead(A1) + micros());
    Serial.begin(115200);
    delay(random(1000, 2000));

    Wire.setSDA(12);
    Wire.setSCL(13);
    Wire1.setSDA(10);
    Wire1.setSCL(11);

    Wire.setClock(100000);
    Wire.begin();             // Initiate as Master
    i2c_address = wake_up();  // Define my Address
    send_id(0x00);            // Broasdcast my ID
    Wire1.setClock(100000);
    Wire1.begin(i2c_address);  // Initiate as Slave
    Wire1.onReceive(recv);
    Wire1.onRequest(req);
}

void loop() {
    // put your main code here, to run repeatedly:
    int i;
    bool finish_flag;
    byte tx_buf[frame_size];
    uint8_t i2c_broadcast_addr = 0x00;

    i2c_msg tx_msg = {i2c_address, '-', '\0', (uint16_t)ID, 0.0f};

    if (STATE == 1) {
        if (ID == 3) {  // Hub makes sure that everybody has all the gains!
            finish_flag = true;
            delay(500);
            for (i = 0; i < NUMBER_OF_RPI; i++) {
                if (i2c_addresses[i] == 0) {
                    Serial.println("Entrei no sem addr");
                    finish_flag = false;
                    tx_msg.value_i = (uint16_t)i + 1;
                    memcpy(tx_buf, &tx_msg, msg_size);
                    Wire.beginTransmission(i2c_broadcast_addr);
                    Wire.write(tx_buf, frame_size);
                    i2c_error_code = Wire.endTransmission();
                    delay(500);
                }
            }
            if (finish_flag == true) {
                Serial.println("cheguei");
                STATE = 2;  // Calibrate State
                HUB_FLAG = true;
            }
        } else
            STATE = 0;  // Idle State
    }

    if (HUB_FLAG == true) {
        Serial.println("ENTREI NO HUB");
        switch (STATE) {
            case 2:
                // Serial.println("STARTING OBTAINING GAINS");
                tx_msg = {i2c_address, '!', '\0', (uint16_t)1, 0.0f};
                memcpy(tx_buf, &tx_msg, msg_size);
                i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);
                delay(100);
                tx_msg = {i2c_address, '!', '\0', (uint16_t)2, 0.0f};
                memcpy(tx_buf, &tx_msg, msg_size);
                i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);
                delay(100);
                tx_msg = {i2c_address, '!', '\0', (uint16_t)3, 0.0f};
                memcpy(tx_buf, &tx_msg, msg_size);
                i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);
                Serial.println("STARTING GAINS");
                Serial.print(gains[0], 6);
                Serial.print(" ");
                Serial.print(gains[1], 6);
                Serial.print(" ");
                Serial.println(gains[2], 6);
                STATE = 0;  // Idle
                break;

            default:
                STATE = 0;  // Idle
                break;
        }
    }

    if (STATE == 0) {
        delay(500);
        Serial.println(
            "--------------------------------------------------------------");
        Serial.print("My Address: ");
        Serial.print(i2c_address, BIN);
        Serial.print("\t My ID: ");
        Serial.print(ID);
        Serial.print("\t My STATE: ");
        Serial.println(STATE);
        Serial.println(
            "--------------------------------------------------------------");
        for (i = 0; i < NUMBER_OF_RPI; i++) {
            Serial.print("Address: ");
            Serial.print(i2c_addresses[i], BIN);
            Serial.print("\tID: ");
            Serial.println(i + 1);
        }
        Serial.println(
            "--------------------------------------------------------------");
    }

    interface();

    delay(1000);
}
