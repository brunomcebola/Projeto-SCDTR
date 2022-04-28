#include "constants.h"
#include "mixin.h"

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

            case 'd':
            case 'r': {
                lum = Serial.read();
                Serial.read();
                val_f = read_number<float>(1);

                Serial.printf("Recebi da linha de cmd: %c", cmd);

                interface_redirect(i2c_addresses[lum - '1'], cmd, '\0', 0,
                                   val_f);

                break;
            }

            case 'g': {
                sub_cmd = Serial.read();
                Serial.read();
                lum = Serial.read();

                if (find_index_char(GETS, N_GETS, sub_cmd) < N_GETS) {
                    interface_redirect(i2c_addresses[lum - '1'], cmd, sub_cmd,
                                       0, 0.0f);
                }

                break;
            }

            case 's': {
                sub_cmd = Serial.read();
                Serial.read();
                lum = Serial.read();

                if (find_index_char(STREAMS, N_STREAMS, sub_cmd) < N_STREAMS) {
                    interface_redirect(i2c_addresses[lum - '1'], cmd, sub_cmd,
                                       0, 0.0f);
                }

                break;
            }

            case 'b': {
                sub_cmd = Serial.read();
                Serial.read();

                if (find_index_char(BUFFER, N_BUFFER, sub_cmd) < N_BUFFER) {
                    lum = Serial.read();

                    interface_redirect(i2c_addresses[lum - '1'], cmd, sub_cmd,
                                       0, 0.0f);
                } else {
                    val_i = read_number<int>(0);

                    interface_redirect(i2c_addresses[sub_cmd - '1'], cmd, '\0',
                                       val_i, 0.0f);
                }

                break;
            }

            break;
        }
    }
}
