void interface_redirect(uint8_t dest_addr, char cmd, char sub_cmd, uint16_t i,
                        float f) {
    byte tx_buf[frame_size];
    i2c_msg tx_msg = {i2c_address, cmd, sub_cmd, i, f};
    memcpy(tx_buf, &tx_msg, msg_size);
    masterTransmission(dest_addr, tx_buf);
}

template <class T>
T read_number(int sel) {
    /*
    SEL:
        0 -> int
        1 -> float
    */

    String input = "";

    while (Serial.available() > 0) {
        char c = Serial.read();

        if (isDigit(c)) {
            input += c;
        } else if (c == '.') {
            if (sel == 1) {
                input += c;
            } else {
                return -1;
            }
        } else if (c == ' ' || c == '\n') {
            if (sel == 0) {
                return input.toInt();
            } else if (sel == 1) {
                return input.toFloat();
            } else {
                return -1;
            }
        } else {
            return -1;
        }
    }

    return -1;
}

void interface() {
    char cmd = '\0';
    char sub_cmd = '\0';
    int val_i = 0;
    float val_f = 0.0f;

    if (Serial.available() > 0) {
        cmd = Serial.read();
        Serial.read();

        switch (cmd) {
            // 'a' - Set anti-windup state at desk i
            // 'o' - Set current occupancy state at desk i
            // 'w' - Set feedforward control state at desk i
            case 'a':
            case 'o':
            case 'w': {
                lum = Serial.read();
                Serial.read();
                val_i = read_number<int>(0);

                interface_redirect(i2c_addresses[lum - '1'], cmd, '\0', val_i,
                                   0.0f);

                break;
            }

            // 'd' - Set duty cycle at luminaire i
            // 'r' - Set illuminance reference at luminaire i
            case 'd':
            case 'r': {
                lum = Serial.read();
                Serial.read();
                val_f = read_number<float>(1);

                interface_redirect(i2c_addresses[lum - '1'], cmd, '\0', 0,
                                   val_f);

                break;
            }

            // Set feedback control state at desk i
            case 'b':
                sub_cmd = Serial.read();
                Serial.read();

                switch (sub_cmd) {
                    case 'd':
                        lum = Serial.read();
                        Serial.read();

                        if (lum == LUMINAIRE) {
                            buffer_duty_cycle = !buffer_duty_cycle;
                            buffer_read_size = buffer.get_used_space();
                            buffer_read_counter = 0;
                            Serial.printf("b d %c ", lum);
                        }

                        break;

                    case 'l':
                        lum = Serial.read();
                        Serial.read();

                        if (lum == LUMINAIRE) {
                            buffer_lux = !buffer_lux;
                            buffer_read_size = buffer.get_used_space();
                            buffer_read_counter = 0;
                            Serial.printf("b l %c ", lum);
                        }

                        break;

                    default:
                        lum = sub_cmd;

                        if (lum == LUMINAIRE) {
                            val_i = read_number<int>(0);

                            controller.set_fb_usage(val_i);

                            Serial.println("ack");

                        } else {
                            Serial.println("err");
                        }

                        break;
                }

                break;

            case 'g':
                sub_cmd = Serial.read();
                Serial.read();

                switch (sub_cmd) {
                    // 'a' - Get anti-windup state at desk i
                    // 'd' - Get current duty cycle at luminaire i
                    // 'l' - Get measured illuminance at luminaire i
                    // 'o' - Get current occupancy state at desk i
                    // 'r' - Get current illuminance reference at luminaire i
                    // 't' - Get elapsed time since last restart
                    // 'w' - Get feedforward control state at desk i
                    case 'a':
                    case 'd':
                    case 'l':
                    case 'o':
                    case 'r':
                    case 't':
                    case 'w': {
                        lum = Serial.read();

                        interface_redirect(i2c_addresses[lum - '1'], cmd,
                                           sub_cmd, 0, 0.0f);
                        break;
                    }

                    // Get feedback control state at desk i
                    case 'b':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("b %c %d\n", lum,
                                          controller.get_fb_usage());
                        }

                        break;

                    // Get accumulated energy consumption at desk i since the
                    // last system restart
                    case 'e':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("e %c %f\n", lum, total_energy);
                        }

                        break;

                    // Get accumulated flicker error at desk i since last system
                    // restart
                    case 'f':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf(
                                "f %c %f\n", lum,
                                flicker_error / (iteration_counter - 2));
                        }

                        break;

                    // Get instantaneous power consumption at desk i
                    case 'p':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf(
                                "p %c %f\n", lum,
                                (controller.get_u() / V_REF) * NOMINAL_POWER);
                        }

                        break;

                    // Get accumulated visibility error at desk i since last
                    // system restart
                    case 'v':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("v %c %f\n", lum,
                                          visibility_error / iteration_counter);
                        }

                        break;

                    // Get current external illuminance at desk i
                    case 'x':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf(
                                "x %c %d\n", lum,
                                ldr_volt_to_lux(n_to_volt(analogRead(A0)), M,
                                                B) -
                                    n_to_lux(volt_to_n(controller.get_u()),
                                             simulator.get_gain()));
                        }

                        break;
                }

                break;

            // Start/Stop stream of real-time variable x of desk i
            case 's':
                sub_cmd = Serial.read();
                Serial.read();

                switch (sub_cmd) {
                    case 'd':
                        lum = Serial.read();
                        Serial.read();

                        if (lum == LUMINAIRE) {
                            stream_duty_cycle = !stream_duty_cycle;
                        }

                        break;

                    case 'l':
                        lum = Serial.read();
                        Serial.read();

                        if (lum == LUMINAIRE) {
                            stream_lux = !stream_lux;
                        }

                        break;
                }

                break;
        }
    }
}