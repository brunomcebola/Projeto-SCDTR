#include <Wire.h>
#include <hardware/flash.h>

#include "consensus.h"

/**/

#include "circular_buffer.h"
#include "constants.h"
#include "controller.h"
#include "mixin.h"
#include "simulator.h"

const int sampling_time = 10;  // ms

Simulator simulator;
Controller controller;
CircularBuffer<6000> buffer;
buffer_data data;

long int initial_time = 0;
long int final_time = 0;

float u_sim = 0;
float u_real = 0;
float u_cont = 0;

char lum = '\0';
char cmd = '\0';
char sub_cmd = '\0';
int val_i = 0;
float val_f = 0.0f;
float duty_cycle = 0.0f;
float lux = 0;
bool stream_duty_cycle = false;
bool stream_lux = false;
bool buffer_duty_cycle = false;
bool buffer_lux = false;
int buffer_read_size = 0;
int buffer_read_counter = 0;

int iteration_counter = 0;
float flicker = 0.0f;

float total_energy = 0.0f;
float visibility_error = 0.0f;
float flicker_error = 0.0f;
float prev_lux_1 = 0.0f;
float prev_lux_2 = 0.0f;

/**/

int ID;
const int NUMBER_OF_RPI = N_RPI;
const int MAX_ITER = 30;
int STATE = 1;  // Wake Up State
int count_iter = 0;
bool HUB_FLAG = false;
/*
 * 0 - Idle
 * 1 - Start/Restart
 * 2 - Calibrate
 */

Consensus consensus(NUMBER_OF_RPI);

// I2C Message Structure
struct i2c_msg {
    uint8_t node;      // source addr
    uint16_t t;        // sample time in ms
    uint16_t value_i;  // sample value (integer)
    char cmd;          // command specification
    char sub_cmd;      // subcommand specification
    uint16_t value_f;
    i2c_msg(uint8_t addr = 0, char c = '\0', char sc = '\0', uint16_t i = 0,
            uint16_t f = 0, uint16_t time = micros())
        : t{time}, node{addr}, cmd{c}, sub_cmd{sc}, value_i{i}, value_f{f} {}
};

// Basic I2C Communication Variables
const int msg_size = sizeof(i2c_msg);
// sometimes the byte is lost in the comms, so we send and extra dummy one
const int frame_size = msg_size + 1;
// reserves 32 slots - increase if more are needed
const int input_fifo_size = 32;
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
float gains[NUMBER_OF_RPI]{1};
uint8_t i2c_addresses[NUMBER_OF_RPI] = {0};
float C = 1.0f;
float O = 0.0f;
float RHO = 0.07f;

float DC[NUMBER_OF_RPI][NUMBER_OF_RPI]{0.0f};
float DC_avg[NUMBER_OF_RPI]{0.0f};
bool DC_full = false;

void setup() {
    delay(10000);
    Serial.begin(115200);
    randomSeed(analogRead(A1) + micros());
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
}

void loop() {
    // put your main code here, to run repeatedly:
    int i;
    bool finish_flag;
    byte tx_buf[frame_size];
    uint8_t i2c_broadcast_addr = 0x00;

    i2c_msg tx_msg = {i2c_address, '-', '\0', (uint16_t)ID};

    //  Serial.println(STATE);

    if (STATE == 1) {
        if (ID == NUMBER_OF_RPI) {  // Hub makes sure that everybody as all the gains!
            finish_flag = true;
            delay(500);
            for (i = 0; i < NUMBER_OF_RPI; i++) {
                if (i2c_addresses[i] == 0) {
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
                tx_msg = {i2c_address, '!', '\0', (uint16_t)1};
                memcpy(tx_buf, &tx_msg, msg_size);
                i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);
                delay(100);
                tx_msg = {i2c_address, '!', '\0', (uint16_t)2};
                memcpy(tx_buf, &tx_msg, msg_size);
                i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);
                delay(100);
                tx_msg = {i2c_address, '!', '\0', (uint16_t)3};
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
                break;
        }
    }
    if (STATE == 3) {
        for (int i = 0; i < NUMBER_OF_RPI; i++) {
            for (int j = 0; j < NUMBER_OF_RPI; j++) {
                DC[i][j] = -1.0;
            }
            DC_avg[i] = 0;
        }

        consensus.iterate();

        byte tx_buf[frame_size];
        i2c_msg tx_msg = {i2c_address, '%'};
        
        while (!DC_full) {
            DC_full = true;
            for (int i = 0; i < NUMBER_OF_RPI; i++) {
                tx_msg.value_i = (uint16_t)i;
                tx_msg.value_f = (uint16_t)((consensus.get_d(i) / 100) * 1000);
                memcpy(tx_buf, &tx_msg, msg_size);
                masterTransmission(0x00, tx_buf);
                delay(3);
            }
            for (int i = 0; i < NUMBER_OF_RPI; i++) {
                for (int j = 0; j < NUMBER_OF_RPI; j++) {
                    if (DC[i][j] == -1.0) {
                        DC_full = false;
                    }
                }
            }
            
            for(int i = 0; i < NUMBER_OF_RPI; i++){
              for(int j= 0; j< NUMBER_OF_RPI;j++){
                Serial.print(DC[i][j]);Serial.print(" ");
              }
              Serial.println();
            }
        }
        for (int i = 0; i < NUMBER_OF_RPI; i++) {
            tx_msg.value_i = (uint16_t)i;
            tx_msg.value_f = (uint16_t)((consensus.get_d(i) / 100) * 1000);
            memcpy(tx_buf, &tx_msg, msg_size);
            masterTransmission(0x00, tx_buf);
            delay(3);
        }

        for (int i = 0; i < NUMBER_OF_RPI; i++) {
            for (int j = 0; j < NUMBER_OF_RPI; j++) {
                DC_avg[i] += DC[i][j];
            }
            DC_avg[i] = DC_avg[i] / NUMBER_OF_RPI;
            consensus.set_d_av(DC_avg[i], i);
        }

        for (int i = 0; i < NUMBER_OF_RPI; i++) {
            consensus.set_y(consensus.get_y(i) + RHO * (consensus.get_d(i) -
                                                        consensus.get_d_av(i)),
                            i);
        }

        count_iter++;
    }

    if (count_iter >= MAX_ITER) {
        STATE = 0;
        controller.set_lux_ref(
            consensus.get_d(0)   * gains[0] +
            consensus.get_d(1)    * gains[1] +
            consensus.get_d(2)    * gains[2]);

        simulator.set_simualtion(
            micros(), analogRead(A0),
            lux_to_n(controller.get_lux_ref(), simulator.get_gain()));

        Serial.print("Duty_cycle: ");
        Serial.print(consensus.get_d(0));Serial.print(" ");
        Serial.print(consensus.get_d(1));Serial.print(" ");
        Serial.println(consensus.get_d(2));
        count_iter = 0;
    }

    // interface to receive comands from the computer
    interface();
}
