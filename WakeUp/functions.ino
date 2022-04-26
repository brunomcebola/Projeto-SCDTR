#include "constants.h"
#include "mixin.h"

void execute_steps(int n_steps, bool loop) {
    const int pwm_steps = ANALOG_MAX/n_steps;

    int pwm, reading;

    Serial.println("\n--- STEPS STARTED ---\n");

    Serial.println("DAC | ADC");

    do {
      for (pwm = 0; pwm <= ANALOG_MAX; pwm += pwm_steps) {
          analogWrite(LED_PIN, pwm);  // sets LED to ADC_write intensity
          delay(1000);

          reading = analogRead(A0);  // read LDR received intensity

          Serial.printf("%d %n\n", pwm, reading);
      }
    } while(loop);    

    Serial.println("\n--- STEPS ENDED ---\n");
}

void execute_comb_step_micro_readings(int n_steps) {
    const int on_samples = 5000;
    const int off_samples = 1000;
    const int pwm_steps = ANALOG_MAX/n_steps;

    int pwm;
    float reading;
    int time_aux = 0;

    for (pwm = 0; pwm <= ANALOG_MAX; pwm += pwm_steps) {
        analogWrite(LED_PIN, 0);  // set led PWM

        for (int i = 0; i < off_samples; i++) {
            time_aux = micros();
            reading = n_to_volt(analogRead(A0));

            Serial.printf("%d %.6f\n", time_aux, reading);
            delay(1);
        }

        analogWrite(LED_PIN, pwm);  // set led PWM

        for (int i = 0; i < on_samples; i++) {
            time_aux = micros();
            reading = n_to_volt(analogRead(A0));

            Serial.printf("%d %.6f\n", time_aux, reading);
        }
    }
}

float calibrate_gain() {
    int x0 = ANALOG_MAX / 8;
    int x1 = ANALOG_MAX;
    float y0, y1, G;

    Serial.println("\n--- GAIN CALIBRATION STARTED ---\n");

    // turns LED on to max intensity
    analogWrite(LED_PIN, ANALOG_MAX);  
    delay(1000);

    // turns LED off
    analogWrite(LED_PIN, 0);  
    delay(1000);

    // sets LED to x0 intensity
    analogWrite(LED_PIN, x0);  
    delay(1000);
    y0 = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);

    // sets LED to x1 intensity
    analogWrite(LED_PIN, x1);  
    delay(1000);
    y1 = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);

    // turns LED off
    analogWrite(LED_PIN, 0);  
    delay(1000);
    
    // gain computation
    G = (y1 - y0) / (x1 - x0);
   
    Serial.println("\n--- GAIN CALIBRATION ENDED ---\n");

    return G;
}

/****************************
*          SECOND PART      *
*****************************/

void calibrateOwnGain(int crossedId){
  int x0 = ANALOG_MAX / 8;
  int x1 = ANALOG_MAX;
  float y0, y1, G;
  Serial.println("ENTREI NOS OWN GANHOS");
  // turns LED on to max intensity
  analogWrite(LED_PIN, ANALOG_MAX);  
  delay(500);

  // turns LED off
  analogWrite(LED_PIN, 0);  
  delay(500);

  // sets LED to x0 intensity
  analogWrite(LED_PIN, x0);  
  delay(500);
  y0 = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);

  // sets LED to x1 intensity
  analogWrite(LED_PIN, x1);  
  delay(500);
  y1 = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);

  // turns LED off
  analogWrite(LED_PIN, 0);  
  delay(500);
    
  // gain computation
  gains[ID - 1] = y1 / x1 ;
}

void calibrateCrossGain(int crossedId){
  int x0 = ANALOG_MAX / 8;
  int x1 = ANALOG_MAX;
  float y0, y1, G;
  Serial.print(crossedId); Serial.print(" ");
  Serial.print("ENTREI NOS GANHOS ");
  Serial.println(micros());
  
  // turns LED on to max intensity
  delay(500);

  // turns LED off
  delay(500);

  // sets LED to x0 intensity
  delay(500);
  y0 = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);

  // sets LED to x1 intensity
  delay(500);
  y1 = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);

  // turns LED off
  delay(500);
    
  // gain computation
  gains[crossedId - 1] = y1 / x1 ;
}

// When Slave sends data do this
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
  
  Serial.print("SOMETHING ");
  Serial.print(msg.command); Serial.print(" ");
  Serial.println(msg.value);

  switch(msg.command){ // Process Information
    
    case '@': // Received Address
      i2c_all_addresses[msg.value-1] = msg.node;
      break;
      
    case '!': // calibration call
        if(msg.value == ID ){
            calibrateOwnGain( msg.value);
        }
        else{
            calibrateCrossGain(  msg.value );
        }
        if(msg.value == 3){
          Serial.println("K VALUES");
          Serial.print(gains[0], 6); Serial.print(" ");
          Serial.print(gains[1], 6); Serial.print(" ");
          Serial.println(gains[2], 6);
        }
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

//NEEDS TO BE CHECKED
int masterTransmission(uint8_t transmission_addr, byte message[]){

  Wire.beginTransmission(transmission_addr);
  Wire.write(message, frame_size);
  return Wire.endTransmission();
}

//NEEDS TO BE CHECKED
int slaveTransmission(uint8_t transmission_addr, byte message[]){

  Wire1.write( message , 12); // ID - 1, because id starts in 1
  return Wire1.endTransmission();
}

void calibrationGain(){
  /*
  char print_buf[200];
  byte tx_buf[frame_size];
  uint8_t i2c_broadcast_addr = 0x00;
  char aux[3];

  // calculte the gains matrix (k)
  for(int i = 1; i <= MAX_IDS; i++){
    aux[0] = '!';
    aux[1] = 48 + i;
    my_i2c_msg tx_msg{ i2c_address, millis(), 0 };
    strcpy(tx_msg.cmd,  aux);

    memcpy(tx_buf, &tx_msg, msg_size);

    masterTransmission(0b0001111 + i, tx_buf);
    delay(2500); // wait seconds, that represent the ammount of time needed for calibration
  }
  // request the vectors for the gains matrix (k)
  */
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
