#include <Wire.h>
#include <hardware/flash.h>

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
// sometimes the last byte is lost in the comms, so we send and extra dummy one
const int frame_size = msg_size + 1;

int n_i2c_errors = 0;
uint8_t i2c_address;  // TODO: rever esta var

// More I2C and System variables
float gains[3]{1, 1, 1};
uint8_t i2c_addresses[NUMBER_OF_RPI] = {0};

void setup() {
    // To ensure we have time to open Serial Monitor
    delay(10000);

    Serial.begin(115200);

    // Randomly delay each node
    randomSeed(analogRead(A1));
    delay(random(1000, 2000));

    // Sets wires' pins
    Wire.setSDA(12);
    Wire.setSCL(13);
    Wire1.setSDA(10);
    Wire1.setSCL(11);

    // Set wires' clocks to be equal
    Wire.setClock(100000);
    Wire1.setClock(100000);

    // Initiate as Master
    Wire.begin();

    // Define my Address
    i2c_address = wake_up();

    // Broasdcast my ID
    send_id(0x00);

    // Initiate as Slave
    Wire1.begin(i2c_address);
    Wire1.onReceive(recv);
    Wire1.onRequest(req);
}

void loop() {
    int i;
    bool finish_flag;
    byte tx_buf[frame_size];
    uint8_t i2c_broadcast_addr = 0x00;

    i2c_msg tx_msg = {i2c_address, '-', '\0', (uint16_t)ID, 0.0f};

    if (STATE == 1) {
        if (ID ==
            NUMBER_OF_RPI) {  // Hub makes sure that everybody as all the gains!
            finish_flag = true;
            delay(500);
            for (i = 0; i < NUMBER_OF_RPI; i++) {
                if (i2c_addresses[i] == 0) {
                    finish_flag = false;
                    tx_msg.value_i = (uint16_t)i + 1;
                    memcpy(tx_buf, &tx_msg, msg_size);
                    Wire.beginTransmission(i2c_broadcast_addr);
                    Wire.write(tx_buf, frame_size);
                    Wire.endTransmission();
                    delay(500);
                }
            }
            if (finish_flag == true) {
                STATE = 2;  // Calibrate State
                HUB_FLAG = true;
            }
        } else
            STATE = 0;  // Idle State
    }

    if (HUB_FLAG == true) {
        switch (STATE) {
            case 2:
                // Serial.println("STARTING OBTAINING GAINS");
                tx_msg = {i2c_address, '!', (uint16_t)1};
                memcpy(tx_buf, &tx_msg, msg_size);
                masterTransmission(i2c_broadcast_addr, tx_buf);
                delay(100);
                tx_msg = {i2c_address, '!', (uint16_t)2};
                memcpy(tx_buf, &tx_msg, msg_size);
                masterTransmission(i2c_broadcast_addr, tx_buf);
                delay(100);
                tx_msg = {i2c_address, '!', (uint16_t)3};
                memcpy(tx_buf, &tx_msg, msg_size);
                masterTransmission(i2c_broadcast_addr, tx_buf);
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
