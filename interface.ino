char lum = '\0';
char cmd = '\0';
char sub_cmd = '\0';

int val_i = 0;
float val_f = 0.0f;

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
                lum = Serial.read();
                Serial.read();

                if (lum == LUMINAIRE) {
                    val_i = read_number<int>(0);

                    controller.set_fb_usage(val_i);

                    Serial.println("ack");

                } else {
                    Serial.println("err");
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

                    // Get measured illuminance at luminaire i
                    case 'l':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf(
                                "l %c %f\n", lum,
                                n_to_lux(analogRead(A0), simulator.get_gain()));
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

                    // Get current illuminance reference at luminaire i
                    case 'r':
                        lum = Serial.read();
                        if (lum == LUMINAIRE) {
                            Serial.printf("r %c %f\n", lum,
                                          controller.get_lux_ref());
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
                }

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