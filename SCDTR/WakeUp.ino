#include <Wire.h>
#include <hardware/flash.h>

#define ID 1 // Change depending on the control node (1, 2 or 3)

// I2C Message Structure
struct my_i2c_msg {
  uint8_t node; //source node
  uint32_t ts; //sample time in ms
  uint16_t value; //sample value
  my_i2c_msg(uint8_t n = 0, uint32_t t = 0, uint16_t v = 0)
  : node {n}, ts{t}, value{v} {}
};

// Basic I2C Communication Variables
const int msg_size = sizeof(my_i2c_msg);
const int frame_size = msg_size + 1; //sometimes the byte is lost in the comms, so we send and extra dummy one
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

void setup() {
  Serial.begin(115200);
  delay(5000);
  flash_get_unique_id(this_pico_id);
  i2c_address = i2c_validate_addr(this_pico_id[7]); // least significant byte of unique ID
  Serial.print("My I2C address: ");
  Serial.println(i2c_address, BIN);
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
  Wire1.setClock(100000);
  Wire1.begin(i2c_address); // Initiate as Slave
  Wire1.onReceive(recv);
  Wire1.onRequest(req);
}

void loop() {
  // put your main code here, to run repeatedly:
  static uint16_t p;
  char print_buf[200];
  byte tx_buf[frame_size];
  uint8_t i2c_broadcast_addr = 0x00;

  my_i2c_msg tx_msg{ i2c_address, millis(), p++};
  
  memcpy(tx_buf, &tx_msg, msg_size);
  
  sprintf(print_buf, "0x%02X TX : %02X , %010lu, %05u\n", i2c_address, tx_msg.node, tx_msg.ts, tx_msg.value );
  Serial.write(print_buf, strlen(print_buf));
  
  Wire.beginTransmission(i2c_broadcast_addr);
  Wire.write(tx_buf, frame_size);
  
  i2c_error_code = Wire.endTransmission();
  if(i2c_error_code != 0) n_i2c_errors++;
  
  sprintf(print_buf, "------- Status: overflows %04d, frame_errors_underrun %04d, frame_errors_overrun %04d, i2c_errors %04d, i2s_error_code %d\n", n_overflows, n_frame_errors_underrun, n_frame_errors_overrun, n_i2c_errors, i2c_error_code );
  Serial.write(print_buf, strlen(print_buf));
  
  if(error_frame_size) {
    Serial.printf("xxxxxxx Error frame size: %d\n", error_frame_size);
    error_frame_size = 0;
  }
  
  delay(1);
  
  for( int i = 0; i < input_fifo_size; i++){
    if(input_slot_full[i]){
      const volatile my_i2c_msg &msg = input_fifo[i];
      sprintf(print_buf, "0x%02X RX : %02X , %010lu, %05u\n",
      i2c_address, msg.node, msg.ts, msg.value );
      input_slot_full[i] = false;
      Serial.write(print_buf, strlen(print_buf));
    }
  }

}

// When Slave sends data do this
void recv(int len) {
  
  int i, n_available_bytes;
  byte rx_buf[frame_size]={0};
  my_i2c_msg msg;

  n_available_bytes = Wire.available();
  if(n_available_bytes != len){
    // Do Something for Debug...
  }
  
  for (i = 0; i < len; i++) rx_buf[i] = Wire1.read();
  
  if(len > frame_size) {
    for (i = frame_size; i < len; i++) Wire1.read(); // Flush
    n_frame_errors_overrun++;
    error_frame_size = len;
  }
  
  if(len < frame_size) {
    n_frame_errors_underrun++;
    error_frame_size = len;
  }
  
  memcpy(&msg, rx_buf, msg_size);
  
  for(i = 0; i < input_fifo_size; i++){
    if(!input_slot_full[i]){
      input_slot_full[i] = true;
      input_fifo[i].node = msg.node;
      input_fifo[i].ts = msg.ts;
      input_fifo[i].value = msg.value;
      break;
    }
  }
  if( i == input_fifo_size ) n_overflows++;
  
}

// When Master requested data do this
void req(void){


  // Do Nothing

  
}

// Confirm validity of the node I2C address
uint8_t i2c_validate_addr(uint8_t unique_id){

  uint8_t id = unique_id + ID;
  
  // make sure it is not a reserved slave address  
  if((id == 0b0001000) || (id == 0b0001100) || (id == 0b1100001)) return (0b0001111 + ID);
  else if((id >= 0b0000000) && (id <= 0b0000111)) return (0b0001111 + ID);
  else if((id >= 0b1110000) && (id <= 0b1111111)) return (0b0001111 + ID);
  return id;
  
}
