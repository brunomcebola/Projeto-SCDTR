#include <Wire.h>
#include <hardware/flash.h>

int ID = 3; // Change depending on the control node (1, 2 or 3)
const int NUMBER_OF_RPI = 3;
int STATE = 1; // Wake Up State
bool HUB_FLAG = false;
/*
 * Describe State values Here
 */

// I2C Message Structure
struct my_i2c_msg {
  uint8_t node; //source node
  uint32_t ts; //sample time in ms
  uint16_t value; //sample value
  char command; //command specification
  my_i2c_msg(uint8_t n = 0, uint32_t t = 0, uint16_t v = 0, char c = 'i')
  : node{n}, ts{t}, value{v}, command{c} {}
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

// More I2C and System variables
float gains[3];
uint8_t i2c_all_addresses[NUMBER_OF_RPI] = {0};

void setup() {

  randomSeed(analogRead(A1));
  Serial.begin(115200);
  delay(random(1000,2000));
  
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
  i2c_address = wake_up(); // Define my Address
  send_id(0x00); // Broasdcast my ID
  Wire1.setClock(100000);
  Wire1.begin(i2c_address); // Initiate as Slave
  Wire1.onReceive(recv);
  Wire1.onRequest(req);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  int i;
  bool finish_flag;
  byte tx_buf[frame_size];
  uint8_t i2c_broadcast_addr = 0x00;

  my_i2c_msg tx_msg = { i2c_address, millis(), (uint16_t) ID, '-'};
  
  if(STATE == 1){
    if(ID == NUMBER_OF_RPI){ // Hub makes sure that everybody as all the gains!
      finish_flag = true;
      delay(500);
      for(i = 0; i < NUMBER_OF_RPI; i++){
        if(i2c_all_addresses[i] == 0){
          finish_flag = false;
          tx_msg.value = (uint16_t) i + 1;
          memcpy(tx_buf, &tx_msg, msg_size);
          Wire.beginTransmission(i2c_broadcast_addr);
          Wire.write(tx_buf, frame_size);
          i2c_error_code = Wire.endTransmission();
          delay(500);
        }
      }
      if(finish_flag == true){
        STATE = 2; // Calibrate State
        HUB_FLAG = true;
      }
    }
    else STATE = 0; // Idle State
  }

  if(HUB_FLAG == true){
    // Act as HUB
  }
  
  Serial.println("--------------------------------------------------"); 
  Serial.print("My Address: "); Serial.print(i2c_address, BIN);
  Serial.print("\t My ID: "); Serial.print(ID);
  Serial.print("\t My STATE: "); Serial.println(STATE); 
  Serial.println("--------------------------------------------------"); 
  for(i = 0; i < NUMBER_OF_RPI; i++){
    Serial.print("Address: "); Serial.print(i2c_all_addresses[i], BIN);
    Serial.print("\tID: "); Serial.println(i+1); 
  }
  Serial.println("--------------------------------------------------"); 
  delay(1000);

}

// When Master sends data do this
void recv(int len) {
  
  int i, n_available_bytes;
  byte rx_buf[frame_size]={0};
  my_i2c_msg msg;

  n_available_bytes = Wire1.available();
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
      input_fifo[i].command = msg.command;
      break;
    }
  }
  if( i == input_fifo_size ) n_overflows++;


  switch(msg.command){ // Process Information
    
    case '@': // Received Address
      i2c_all_addresses[msg.value-1] = msg.node;
      break;
      
    case '!': // Callibration
      break;
      
    case '-': // Ask to Broadcast ID    
      if(msg.value == ID) send_id(0x00); // Broasdcast my ID
      break;
      
    default:
      Serial.println("err");
      break;
       
  }
  
}

// When Master requested data do this
void req(void){

  Serial.println("Here!");

  char print_buf[200];
  byte tx_buf[frame_size];
  uint8_t i2c_broadcast_addr = 0x00;

  my_i2c_msg tx_msg{ i2c_address, millis(), 0};
  
  memcpy(tx_buf, &tx_msg, msg_size);
  
  Wire1.write(tx_buf, frame_size);
  
}

// Confirm validity of the node I2C address
bool i2c_validate_addr(uint8_t id){

  bool flag = true;
  if((id == 0b0001000) || (id == 0b0001100) || (id == 0b1100001)) flag = false;
  else if((id >= 0b0000000) && (id <= 0b0000111)) flag = false;
  else if((id >= 0b1110000) && (id <= 0b1111111)) flag = false;
  return flag;
  
}

// Wake Up and Define Node Address
uint8_t wake_up(void){

  uint8_t addr = 0b0001111;
  int k=0;

  // Address range: 16 - 111
  do{
    
    addr++;
    //if(i2c_validate_addr(addr) == false) continue;
    k++;
    
    do{
      Wire.beginTransmission(addr);
      i2c_error_code = Wire.endTransmission(); 
      delay(1);
    }while(i2c_error_code == 4); // Make sure that it can talk

    
  }while((i2c_error_code == 0) && (addr < 0b1110000));
  // Once an open address is found, use that address

  ID = k;
  return addr;
  
}

// Broadcast my ID
void send_id(uint8_t send_addr){

  byte tx_buf[frame_size];

  my_i2c_msg tx_msg = { i2c_address, millis(), (uint16_t) ID, '@'};
  
  memcpy(tx_buf, &tx_msg, msg_size);

  Wire.beginTransmission(send_addr);
  Wire.write(tx_buf, frame_size);
  i2c_error_code = Wire.endTransmission();

  if(i2c_error_code != 0) n_i2c_errors++;
  
  i2c_all_addresses[ID-1] = i2c_address;
  
}
