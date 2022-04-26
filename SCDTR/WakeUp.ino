#include <Wire.h>
#include <hardware/flash.h>
#include "constants.h"

int ID = 1; // Change depending on the control node (1, 2 or 3)

// I2C Message Structure
struct my_i2c_msg {
  uint8_t node; //source node
  uint32_t ts; //sample time in ms
  char value[3]; //sample value
  my_i2c_msg(uint8_t n = 0, uint32_t t = 0, uint16_t v = 0)
  : node {n}, ts{t}, value{v} {}
};

// Basic I2C Communication Variables
const int msg_size = sizeof(my_i2c_msg);
const int frame_size = msg_size + 1; //sometimes the byte is lost in the comms, so we send an extra dummy one
const int input_fifo_size = 32; //reserves 32 slots - increase if more are needed
volatile my_i2c_msg input_fifo[input_fifo_size];
volatile bool input_slot_full[input_fifo_size] {false};
volatile int n_frame_errors_underrun = 0;
volatile int n_frame_errors_overrun = 0;
volatile int n_overflows = 0;
volatile int error_frame_size;
int i2c_error_code = 0;            
int n_i2c_errors = 0;
uint8_t this_pico_id[8];
uint8_t i2c_address;

uint8_t i2c_all_addresses[NUMBER_OF_RPI] = {0};

float k[3][3]{0};

void setup() {
  Serial.begin(115200);
  delay(5000);
  if(ID == 3){
    Wire.setSDA(20);  // (1,2) - PIN 12 | (3) - PIN 20
    Wire.setSCL(21);  // (1,2) - PIN 13 | (3) - PIN 21
    Wire1.setSDA(18); // (1,2) - PIN 10 | (3) - PIN 18
    Wire1.setSCL(19); // (1,2) - PIN 11 | (3) - PIN 19
  }
  else{
    Wire.setSDA(12);  // (1,2) - PIN 12 | (3) - PIN 20
    Wire.setSCL(13);  // (1,2) - PIN 13 | (3) - PIN 21
    Wire1.setSDA(10); // (1,2) - PIN 10 | (3) - PIN 18
    Wire1.setSCL(11); // (1,2) - PIN 11 | (3) - PIN 19
  }

  Wire.setClock(100000);
  Wire.begin(); // Initiate as Master
  i2c_address = wake_up();
  // flash_get_unique_id(this_pico_id);
  // i2c_address = i2c_validate_addr(flash_get_unique_id); // least significant byte of unique ID
  Serial.print("My I2C address: ");
  Serial.println(i2c_address, BIN);
  
  Wire1.setClock(100000);
  Wire1.begin(i2c_address); // Initiate as Slave
  Wire1.onReceive(recv);
  Wire1.onRequest(req);


  // put your main code here, to run repeatedly:
  static uint16_t p;
  char print_buf[200];
  byte tx_buf[frame_size];
  uint8_t i2c_broadcast_addr = 0x00;

  my_i2c_msg tx_msg{ i2c_address, millis(), '»'};
  
  memcpy(tx_buf, &tx_msg, msg_size);
  
  sprintf(print_buf, "0x%02X TX : %02X , %010lu, %05u\n", i2c_address, tx_msg.node, tx_msg.ts, tx_msg.value );
  Serial.write(print_buf, strlen(print_buf));
  
  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  if(i2c_error_code != 0) n_i2c_errors++;
  
  sprintf(print_buf, "------- Status: overflows %04d, frame_errors_underrun %04d, frame_errors_overrun %04d, i2c_errors %04d, i2s_error_code %d\n", n_overflows, n_frame_errors_underrun, n_frame_errors_overrun, n_i2c_errors, i2c_error_code );
  Serial.write(print_buf, strlen(print_buf));
  
  if(error_frame_size) {
    Serial.printf("xxxxxxx Error frame size: %d\n", error_frame_size);
    error_frame_size = 0;
  }
  
  delay(1000);
  
  for( int i = 0; i < input_fifo_size; i++){
    if(input_slot_full[i]){
      const volatile my_i2c_msg &msg = input_fifo[i];
      sprintf(print_buf, "0x%02X RX : %02X , %010lu, %05u\n",
      i2c_address, msg.node, msg.ts, msg.value );
      input_slot_full[i] = false;
      Serial.write(print_buf, strlen(print_buf));
    }
  }

  tx_msg = { i2c_address, millis(), '!1'};
  memcpy(tx_buf, &tx_msg, msg_size);
  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  tx_msg = { i2c_address, millis(), '!2'};
  memcpy(tx_buf, &tx_msg, msg_size);
  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  tx_msg = { i2c_address, millis(), '!3'};
  memcpy(tx_buf, &tx_msg, msg_size);
  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  //example, since i don't know if we want to fix the center.
  //Wire.requestFrom(i2c_all_adresses[0], 12) // vector of 3 floats <- quantity
  //Wire.requestFrom(i2c_all_adresses[1], 12) // vector of 3 floats <- quantity

}

void loop() {
  

}

// Confirm validity of the node I2C address
uint8_t i2c_validate_addr(uint8_t input){

  bool flag;
  uint8_t id = input;
  int i;
  
  // make sure it is not a reserved slave address  
  do{
    flag = true;
    if((id == 0b0001000) || (id == 0b0001100) || (id == 0b1100001)) flag = false;
    else if((id >= 0b0000000) && (id <= 0b0000111)) flag = false;
    else if((id >= 0b1110000) && (id <= 0b1111111)) flag = false;
    for(i = 0; i < NUMBER_OF_RPI; i++){
      if(id == i2c_all_addresses[i]) flag = false;
    }
    if(flag == false) id = id + 1;
  }while(flag == false);

  return id;
  
}

// Wake Up and Define Node Address
uint8_t wake_up(void){

  uint8_t new_addr, i2c_broadcast_addr = 0x00;
  int n_recv, n_read, i, k;
  byte rx_buf[frame_size]={0};
  my_i2c_msg msg;
  
  n_recv = Wire.requestFrom(i2c_broadcast_addr, frame_size, true);

  delay(1);
  
  n_read = Wire.available();
  if(n_read == 0){
    while(n_read > 0){
      for (i = 0; i < frame_size; i++){
        rx_buf[i] = Wire.read();
        n_read--;
      }
      memcpy(&msg, rx_buf, msg_size);
      i2c_all_addresses[msg.value-1] = msg.node;
    }
    k = 0;
    while(i2c_all_addresses[k] == 0) k++;
    ID = k+1;
    i2c_all_addresses[k] = i2c_validate_addr(random(0b001000,0b1101111));
    new_addr = i2c_all_addresses[k];   
  }
  else{
    i2c_all_addresses[0] = i2c_validate_addr(random(0b001000,0b1101111));
    new_addr = i2c_all_addresses[0];
  }

  return new_addr;
  
}
