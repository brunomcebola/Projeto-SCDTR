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

void calibrateOwnGain(){
  int x0 = ANALOG_MAX / 8;
  int x1 = ANALOG_MAX;
  float y0, y1, G;
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
  k[id - 1][id - 1] = (y1 - y0) / (x1 - x0);
}

void calibrateCrossGain(int crossedId){
  int x0 = ANALOG_MAX / 8;
  int x1 = ANALOG_MAX;
  float y0, y1, G;
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
  k[id - 1][crossedId] = (y1 - y0) / (x1 - x0);
}

// When Slave sends data do this
void recv(int len) {
  
  int i, n_available_bytes;
  byte rx_buf[frame_size]={0};
  my_i2c_msg msg;

  n_available_bytes = Wire.available();
  if(n_available_bytes != len){
    // Do Something for Debug...
    if(len > frame_size) {
    for (i = frame_size; i < len; i++) Wire1.read(); // Flush
    n_frame_errors_overrun++;
    error_frame_size = len;
    }
    
    if(len < frame_size) {
        n_frame_errors_underrun++;
        error_frame_size = len;
    }
  }
  
  //storing msg recieved in the input_fifo
  for (i = 0; i < len; i++) rx_buf[i] = Wire1.read();
  
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

  switch (msg.value[0]){
    case '!': // calibration call
        if(msg.value[1] == (char) (48 + ID) ){
            calibrateOwnGain();
        }
        else{
            calibrateCrossGain(int msg.value[1].toInt());
        }
      break;
    case '»':
      Serial.print("STARTING THE HUB");
      break;  
    default:
      Serial.print("ERROR ON RECEIVING CALL, COMMAND NOT FOUND");
      break;
  }  
}

// When Master requested data do this
void req(void){


  // Do Nothing

  
}

//NEEDS TO BE CHECKED
int masterTransmission(uint8_t transmission_addr, byte message[]){

  Wire.beginTransmission(i2c_broadcast_addr);
  Wire.write(message, frame_size);
  Wire.endTransmission();
}

//NEEDS TO BE CHECKED
int slaveTransmission(uint8_t transmission_addr, byte message[]){
  Wire1.onRequest(req);
  Wire1.write( k[ID - 1] ); // ID - 1, because id starts in 1
  return Wire1.endTransmission();
}

void calibrationGain(){
  char print_buf[200];
  byte tx_buf[frame_size];
  uint8_t i2c_broadcast_addr = 0x00;

  // calculte the gains matrix (k)
  for(int i = 1; i <= MAX_IDS; i++){
    my_i2c_msg tx_msg{ i2c_address, millis(), '!' + (char) (48 + i) };
  
    memcpy(tx_buf, &tx_msg, msg_size);

    masterTransmission(0b0001111 + i, tx_buf);
    delay(1000); // wait 1 second before transmitting another message;
  }
  // request the vectors for the gains matrix (k)

}