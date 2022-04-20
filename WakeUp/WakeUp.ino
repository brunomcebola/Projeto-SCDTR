#include <Wire.h>
#include <hardware/flash.h>

#define ID 3 // Change depending on the control node (1, 2 or 3)
const int NUMBER_OF_RPI = 3;

// I2C Message Structure
struct my_i2c_msg {
  uint8_t node; //source node
  uint32_t ts; //sample time in ms
  uint16_t value; //sample value
  String cmd;
  my_i2c_msg(uint8_t n = 0, uint32_t t = 0, uint16_t v = 0, String c = " ")
    : node {n}, ts{t}, value{v}, cmd{c} {}
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

uint8_t i2c_all_adresses[NUMBER_OF_RPI] = {0b0001111 + 1, 0b0001111 + 2 , 0b0001111 + 3};

float k[3][3] {0};

void setup() {
  Serial.begin(115200);
  delay(5000);
  flash_get_unique_id(this_pico_id);
  i2c_address = i2c_validate_addr(); // least significant byte of unique ID
  Serial.print("My I2C address: ");
  Serial.println(i2c_address, BIN);

  //this is only for joao

  Wire.setSDA(12);
  Wire.setSCL(13);
  Wire1.setSDA(10);
  Wire1.setSCL(11);

  Wire.setClock(100000);
  Wire.begin(); // Initiate as Master

  Wire1.setClock(100000);
  Wire1.begin(i2c_address); // Initiate as Slave
  Wire1.onReceive(recv);
  Wire1.onRequest(req);


  // put your main code here, to run repeatedly:
  static uint16_t p;
  char print_buf[200];
  byte tx_buf[frame_size];
  uint8_t i2c_broadcast_addr = 0x00;
  byte message_requested[12];

  my_i2c_msg tx_msg{ i2c_address, millis(), 0 , "?"};

  memcpy(tx_buf, &tx_msg, msg_size);

  sprintf(print_buf, "0x%02X TX : %02X , %010lu, %05u\n", i2c_address, tx_msg.node, tx_msg.ts, tx_msg.value );
  Serial.write(print_buf, strlen(print_buf));

  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  if (i2c_error_code != 0) n_i2c_errors++;

  sprintf(print_buf, "------- Status: overflows %04d, frame_errors_underrun %04d, frame_errors_overrun %04d, i2c_errors %04d, i2s_error_code %d\n", n_overflows, n_frame_errors_underrun, n_frame_errors_overrun, n_i2c_errors, i2c_error_code );
  Serial.write(print_buf, strlen(print_buf));

  if (error_frame_size) {
    Serial.printf("xxxxxxx Error frame size: %d\n", error_frame_size);
    error_frame_size = 0;
  }

  delay(1000);

  for ( int i = 0; i < input_fifo_size; i++) {
    if (input_slot_full[i]) {
      const volatile my_i2c_msg &msg = input_fifo[i];
      sprintf(print_buf, "0x%02X RX : %02X , %010lu, %05u\n",
              i2c_address, msg.node, msg.ts, msg.value );
      input_slot_full[i] = false;
      Serial.write(print_buf, strlen(print_buf));
    }
  }

  tx_msg.cmd = "!1";
  tx_msg.ts = millis();
  memcpy(tx_buf, &tx_msg, msg_size);
  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  tx_msg.cmd = "!2";
  tx_msg.ts = millis();
  memcpy(tx_buf, &tx_msg, msg_size);
  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  tx_msg.cmd = "!3";
  tx_msg.ts = millis();
  memcpy(tx_buf, &tx_msg, msg_size);
  i2c_error_code = masterTransmission(i2c_broadcast_addr, tx_buf);

  //example, since i don't know if we want to fix the center.
  Wire.requestFrom(i2c_all_adresses[0], 12); // vector of 3 floats <- quantity
  memcpy(message_requested, k[0] , 12);
  slaveTransmission(i2c_all_adresses[2], message_requested);


  Wire.requestFrom(i2c_all_adresses[1], 12); // vector of 3 floats <- quantity
  memcpy(message_requested, k[1] , 12);
  slaveTransmission(i2c_all_adresses[2], message_requested);

  Serial.print(k[0][0], 6); Serial.print(" ");
  Serial.print(k[0][1], 6); Serial.print(" ");
  Serial.println(k[0][2], 6);

  Serial.print(k[1][0], 6); Serial.print(" ");
  Serial.print(k[1][1], 6); Serial.print(" ");
  Serial.println(k[1][2], 6);

  Serial.print(k[2][0], 6); Serial.print(" ");
  Serial.print(k[2][1], 6); Serial.print(" ");
  Serial.println(k[2][2], 6);

}

void loop() {


}

// Confirm validity of the node I2C address
uint8_t i2c_validate_addr() {

  uint8_t id = 0b0001111 + ID;

  // make sure it is not a reserved slave address
  if ((id == 0b0001000) || (id == 0b0001100) || (id == 0b1100001)) return (0b0001111 + ID);
  else if ((id >= 0b0000000) && (id <= 0b0000111)) return (0b0001111 + ID);
  else if ((id >= 0b1110000) && (id <= 0b1111111)) return (0b0001111 + ID);
  return id;

}
