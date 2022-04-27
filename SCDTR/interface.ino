/*
template <class T>
T read_number(int sel) {

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
    if (Serial.available() > 0) {
        cmd = Serial.read();
        Serial.read();

        switch (cmd) {
            // Set anti-windup state at desk i
            case 'a':
                lum = Serial.read();
                Serial.read();

                if (lum == LUMINAIRE) {
                    val_i = read_number<int>(0);

                    controller.set_anti_wind_up_usage(val_i);

                    Serial.println("ack");

                } else {
                    Serial.println("err");
                }

                break;

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

            // Set duty cycle at luminaire i
            case 'd':
                lum = Serial.read();
                Serial.read();

                if (lum == LUMINAIRE) {
                    val_f = read_number<float>(1);

                    analogWrite(LED_PIN, val_f * ANALOG_MAX);

                    Serial.println("ack");

                } else {
                    Serial.println("err");
                }

                break;

            case 'g':
                sub_cmd = Serial.read();
                Serial.read();

                switch (sub_cmd) {
                    // Get anti-windup state at desk i
                    case 'a':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("a %c %d\n", lum,
                                          controller.get_anti_wind_up_usage());
                        }

                        break;

                    // Get feedback control state at desk i
                    case 'b':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("b %c %d\n", lum,
                                          controller.get_fb_usage());
                        }

                        break;

                    // Get current duty cycle at luminaire i
                    case 'd':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("d %c %f\n", lum,
                                          controller.get_u() / V_REF);
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

                    // Get measured illuminance at luminaire i
                    case 'l':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("l %c %f\n", lum,
                                          ldr_volt_to_lux(
                                              n_to_volt(analogRead(A0)), M, B));
                        }

                        break;

                    // Get current occupancy state at desk i
                    case 'o':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("o %c %d\n", lum,
                                          controller.get_occupancy());
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

                    // Get current illuminance reference at luminaire i
                    case 'r':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("r %c %f\n", lum,
                                          controller.get_lux_ref());
                        }

                        break;

                    // Get elapsed time since last restart
                    case 't':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("t %c %.2f\n", lum,
                                          micros() * pow(10, -6));
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

                    // Get feedforward control state at desk i
                    case 'w':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("w %c %d\n", lum,
                                          controller.get_ff_usage());
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

            // Set current occupancy state at desk i
            case 'o':
                lum = Serial.read();
                Serial.read();

                if (lum == LUMINAIRE) {
                    val_i = read_number<int>(0);

                    controller.set_occupancy(val_i);

                    Serial.println("ack");

                } else {
                    Serial.println("err");
                }

                break;

            // Set illuminance reference at luminaire i
            case 'r':
                lum = Serial.read();
                Serial.read();

                if (lum == LUMINAIRE) {
                    val_f = read_number<float>(1);

                    controller.set_lux_ref(val_f);

                    simulator.set_simualtion(micros(), analogRead(A0),
                                             lux_to_n(controller.get_lux_ref(),
                                                      simulator.get_gain()));

                    Serial.println("ack");

                } else {
                    Serial.println("err");
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

            // Set feedforward control state at desk i
            case 'w':
                lum = Serial.read();
                Serial.read();

                if (lum == LUMINAIRE) {
                    val_i = read_number<int>(0);

                    controller.set_ff_usage(val_i);

                    Serial.println("ack");

                } else {
                    Serial.println("err");
                }

                break;
        }
    }
}
*/
