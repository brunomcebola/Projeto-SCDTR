#include "constants.h"
#include "mixin.h"

void execute_steps(int n_steps, bool loop) {
    const int pwm_steps = ANALOG_MAX / n_steps;

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
    } while (loop);

    Serial.println("\n--- STEPS ENDED ---\n");
}

void execute_comb_step_micro_readings(int n_steps) {
    const int on_samples = 5000;
    const int off_samples = 1000;
    const int pwm_steps = ANALOG_MAX / n_steps;

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

void calibrateOwnGain(int crossedId) {
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
    gains[ID - 1] = y1 / x1;
    gains[ID - 1] = gains[ID - 1] / 100 * 4096;
}

void calibrateCrossGain(int crossedId) {
    int x0 = ANALOG_MAX / 8;
    int x1 = ANALOG_MAX;
    float y0, y1, G;
    Serial.print(crossedId);
    Serial.print(" ");
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
    gains[crossedId - 1] = y1 / x1;
    gains[crossedId - 1] = gains[crossedId - 1] / 100 * 4096;
}

// When Slave sends data do this
void recv(int len) {
    int i, n_available_bytes;
    byte rx_buf[frame_size] = {0};
    i2c_msg msg;
    int lum;

    n_available_bytes = Wire1.available();
    if (n_available_bytes != len) {
        // Do Something for Debug...
    }

    for (i = 0; i < len; i++) rx_buf[i] = Wire1.read();

    if (len > frame_size) {
        for (i = frame_size; i < len; i++) Wire1.read();  // Flush
        n_frame_errors_overrun++;
        error_frame_size = len;
    }

    if (len < frame_size) {
        n_frame_errors_underrun++;
        error_frame_size = len;
    }

    memcpy(&msg, rx_buf, msg_size);

    Serial.print("SOMETHING ");
    Serial.print(msg.cmd);
    Serial.print(" ");
    Serial.println(msg.value_i);

    switch (msg.cmd) {  // Process Information

        // Received Address
        case '@': {
            i2c_addresses[msg.value_i - 1] = msg.node;
            break;
        }

        // calibration call
        case '!': {
            if (msg.value_i == ID) {
                calibrateOwnGain(msg.value_i);
            } else {
                calibrateCrossGain(msg.value_i);
            }
            if (msg.value_i == 3) {
                Serial.println("K VALUES");
                Serial.print(gains[0], 6);
                Serial.print(" ");
                Serial.print(gains[1], 6);
                Serial.print(" ");
                Serial.println(gains[2], 6);
            }
            break;
        }

        // Ask to Broadcast ID
        case '-': {
            if (msg.value_i == ID) send_id(0x00);  // Broasdcast my ID
            break;
        }

        // Signal to start consensus
        case ',': {
            DC_full = false;
            count_iter = 0;
            consensus.defining(ID, gains, C, O, controller.get_lux_ref(), RHO);
            STATE = 3;
            break;
        }

        // parse duty cycles from consensus
        case '%': {
            lum = find_index_int(i2c_addresses, NUMBER_OF_RPI, msg.node);
            Serial.print("% ");Serial.println(lum);
            DC[lum][msg.value_i] = (msg.value_f / 1000) * 100;
            break;
        }

        // Interface calls
        case 'a': {
            set_anti_windup_state(msg.value_i);
            break;
        }

        case 'b': {
            switch (msg.sub_cmd) {
                case 'd':
                    buffer_duty_cycle = !buffer_duty_cycle;
                    buffer_lux = false;
                    buffer_read_size = buffer.get_used_space();
                    buffer_read_counter = 0;

                    Serial.printf("b d %c ", lum);

                    break;

                case 'l':
                    buffer_lux = !buffer_lux;
                    buffer_duty_cycle = false;
                    buffer_read_size = buffer.get_used_space();
                    buffer_read_counter = 0;

                    Serial.printf("b l %c ", lum);

                    break;

                default:
                    set_feedback_state(msg.value_i);
                    break;
            }

            break;
        }

        case 'c': {
            set_cost(msg.value_f / 1000);
            break;
        }

        case 'd': {
            set_instantaneous_duty_cycle(msg.value_f / 1000);
            break;
        }

        case 'o': {
            set_occupancy_state(msg.value_i);
            break;
        }

        case 'r': {
            set_lux_ref(msg.value_f / 1000);
            break;
        }

        case 'w': {
            set_feedforward_state(msg.value_i);
            break;
        }

        case 'g': {
            switch (msg.sub_cmd) {
                case 'a':
                case 'b':
                case 'o':
                case 'w': {
                    send_int(msg.sub_cmd);
                    break;
                }

                case 'c':
                case 'd':
                case 'e':
                case 'f':
                case 'l':
                case 'p':
                case 'r':
                case 't':
                case 'v':
                case 'x':
                case 'L':
                case 'O':
                case 'U': {
                    send_float(msg.sub_cmd);
                    break;
                }
            }
            break;
        }

        case 's': {
            switch (msg.sub_cmd) {
                case 'd':
                    stream_duty_cycle = !stream_duty_cycle;
                    stream_lux = false;
                    break;

                case 'l':
                    stream_lux = !stream_lux;
                    stream_duty_cycle = false;
                    break;
            }
        }

        case 'O': {
            set_lower_bound_for_occupied(msg.value_f / 1000);
            break;
        }

        case 'U': {
            set_lower_bound_for_unoccupied(msg.value_f / 1000);
            break;
        }

        // Interface responses
        case '?': {
            lum = find_index_int(i2c_addresses, NUMBER_OF_RPI, msg.node);

            switch (msg.sub_cmd) {
                case 'a':
                case 'b':
                case 'o':
                case 'w': {
                    print_int(msg.sub_cmd, lum, msg.value_i);
                    break;
                }

                case 'c':
                case 'd':
                case 'e':
                case 'f':
                case 'l':
                case 'p':
                case 'r':
                case 't':
                case 'v':
                case 'x':
                case 'L':
                case 'O':
                case 'U': {
                    print_float(msg.sub_cmd, lum, msg.value_f / 1000);
                    break;
                }
            }
            break;
        }

        // Stream
        case '*': {
            lum = find_index_int(i2c_addresses, NUMBER_OF_RPI, msg.node);
            print_stream(msg.sub_cmd, lum, msg.value_f / 1000, msg.t);
            break;
        }

        // Buffer
        case '$': {
            lum = find_index_int(i2c_addresses, NUMBER_OF_RPI, msg.node);
            print_buffer(msg.sub_cmd, lum, msg.value_i, msg.value_f / 1000);
        }
    }
}

// NEEDS TO BE CHECKED
int masterTransmission(uint8_t transmission_addr, byte message[]) {
    Wire.beginTransmission(transmission_addr);
    Wire.write(message, frame_size);
    return Wire.endTransmission();
}

// NEEDS TO BE CHECKED
int slaveTransmission(uint8_t transmission_addr, byte message[]) {
    Wire1.write(message, 12);  // ID - 1, because id starts in 1
    return Wire1.endTransmission();
}

// Confirm validity of the node I2C address
bool i2c_validate_addr(uint8_t id) {
    bool flag = true;
    if ((id == 0b0001000) || (id == 0b0001100) || (id == 0b1100001))
        flag = false;
    else if ((id >= 0b0000000) && (id <= 0b0000111))
        flag = false;
    else if ((id >= 0b1110000) && (id <= 0b1111111))
        flag = false;
    return flag;
}

// Wake Up and Define Node Address
uint8_t wake_up(void) {
    uint8_t addr = 0b0001111;
    int k = 0;

    // Address range: 16 - 111
    do {
        addr++;
        // if(i2c_validate_addr(addr) == false) continue;
        k++;

        do {
            Wire.beginTransmission(addr);
            i2c_error_code = Wire.endTransmission();
            delay(1);
        } while (i2c_error_code == 4);  // Make sure that it can talk

    } while ((i2c_error_code == 0) && (addr < 0b1110000));
    // Once an open address is found, use that address

    ID = k;
    return addr;
}

// Broadcast my ID
void send_id(uint8_t send_addr) {
    byte tx_buf[frame_size];

    i2c_msg tx_msg = {i2c_address, '@', '\0', (uint16_t)ID};

    memcpy(tx_buf, &tx_msg, msg_size);

    Wire.beginTransmission(send_addr);
    Wire.write(tx_buf, frame_size);
    i2c_error_code = Wire.endTransmission();

    if (i2c_error_code != 0) n_i2c_errors++;

    i2c_addresses[ID - 1] = i2c_address;
}

/* INTERFACE */

// set variables
void set_instantaneous_duty_cycle(float dc) {
    analogWrite(LED_PIN, duty_cycle * ANALOG_MAX);
}

void set_lux_ref(float lux_ref) {
    /*controller.set_lux_ref(lux_ref);

    simulator.set_simualtion(
        micros(), analogRead(A0),
        lux_to_n(controller.get_lux_ref(), simulator.get_gain()));*/

    byte tx_buf[frame_size];
    i2c_msg tx_msg = {i2c_address, ','};

    memcpy(tx_buf, &tx_msg, msg_size);
    masterTransmission(0x00, tx_buf);
    DC_full = false;
    count_iter = 0;
    consensus.defining(ID, gains, C, O, lux_ref , RHO);
    STATE = 3;
}

void set_occupancy_state(bool state) { 
  controller.set_occupancy(state);
  consensus.set_occupation(state);  
}

void set_anti_windup_state(bool state) {
    controller.set_anti_wind_up_usage(state);
}

void set_feedforward_state(bool state) { controller.set_ff_usage(state); }

void set_feedback_state(bool state) { controller.set_fb_usage(state); }

void set_lower_bound_for_occupied(float duty_cycle) {
    consensus.set_oboundary(duty_cycle);
}

void set_lower_bound_for_unoccupied(float duty_cycle) {
    consensus.set_uboundary(duty_cycle);
}

void set_cost(float lux_ref) { consensus.set_cost(lux_ref); }

// send responses
void send_float(char sub_cmd) {
    float val;

    switch (sub_cmd) {
        case 'c': {
            consensus.get_cost();
            break;
        }
        case 'd': {
            val = controller.get_u() / V_REF;
            break;
        }
        case 'e': {
            val = total_energy;
            break;
        }
        case 'f': {
            val = flicker_error / (iteration_counter - 2);
            break;
        }
        case 'l': {
            val = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B);
            break;
        }
        case 'p': {
            val = ((controller.get_u() / V_REF) * NOMINAL_POWER);
            break;
        }
        case 'r': {
            val = controller.get_lux_ref();
            break;
        }
        case 't': {
            val = micros() * pow(10, -6);
            break;
        }
        case 'v': {
            val = visibility_error / iteration_counter;
            break;
        }
        case 'x': {
            val = ldr_volt_to_lux(n_to_volt(analogRead(A0)), M, B) -
                  n_to_lux(volt_to_n(controller.get_u()), simulator.get_gain());
            break;
        }
        case 'L': {
            if (controller.get_occupancy()) {
                val = consensus.get_boundary_occupied();
            } else {
                val = consensus.get_boundary_unoccupied();
            }

            break;
        }
        case 'O': {
            val = consensus.get_boundary_occupied();
            break;
        }
        case 'U': {
            val = consensus.get_boundary_unoccupied();
            break;
        }
    }

    byte tx_buf[frame_size];
    i2c_msg tx_msg = {i2c_address, '?', sub_cmd, 0, (uint16_t)(val * 1000)};

    memcpy(tx_buf, &tx_msg, msg_size);
    masterTransmission(i2c_addresses[2], tx_buf);
}

void send_int(char sub_cmd) {
    int val;

    switch (sub_cmd) {
        case 'a': {
            val = controller.get_anti_wind_up_usage();
            break;
        }
        case 'b': {
            val = controller.get_fb_usage();
            break;
        }
        case 'o': {
            val = controller.get_occupancy();
            break;
        }
        case 'w': {
            val = controller.get_ff_usage();
            break;
        }
    }

    byte tx_buf[frame_size];
    i2c_msg tx_msg = {i2c_address, '?', sub_cmd, (uint16_t)val};
    memcpy(tx_buf, &tx_msg, msg_size);
    masterTransmission(i2c_addresses[2], tx_buf);
}

void send_stream(char sub_cmd, float val) {
    byte tx_buf[frame_size];
    i2c_msg tx_msg = {i2c_address, '*', sub_cmd, 0, (uint16_t)(val * 1000)};

    memcpy(tx_buf, &tx_msg, msg_size);
    masterTransmission(i2c_addresses[2], tx_buf);
}

void send_buffer(char sub_cmd, float val, int iter) {
    byte tx_buf[frame_size];
    i2c_msg tx_msg = {i2c_address, '$', sub_cmd, (uint16_t)iter,
                      (uint16_t)(val * 1000)};

    memcpy(tx_buf, &tx_msg, msg_size);
    masterTransmission(i2c_addresses[2], tx_buf);
}

// print responses
void print_int(char cmd, int lum, int val) {
    Serial.printf("%c %d %d\n", cmd, lum, val);
}

void print_float(char cmd, int lum, float val) {
    Serial.printf("%c %d %f\n", cmd, lum, val);
}

void print_stream(char sub_cmd, int lum, float val, int t) {
    Serial.printf("s %c %d %f %d\n", sub_cmd, lum, val, t);
}

void print_buffer(char sub_cmd, int lum, int iter, float val) {
    if (iter == 0) {
        Serial.printf("b %c %d ", sub_cmd, lum);
    }
    Serial.printf("%f, ", val);
}