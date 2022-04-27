#ifndef CONSTANTS_H
#define CONSTANTS_H

const int R2 = 10000;
const int LED_PIN = 2;
const float V_REF = 3.3;
const int ANALOG_MAX = 4096;
const int ANALOG_FREQ = 60000;

const float M = -0.9;
const float B = 6.2;

const char LUMINAIRE = '1';

const float NOMINAL_POWER = 1;

const short int MAX_IDS = 3;

const int N_GETS = 13;
const char GETS[N_GETS] = {'a', 'b', 'd', 'e', 'f', 'l', 'o',
                           'p', 'r', 't', 'v', 'w', 'x'};

const int N_STREAMS = 2;
const char STREAMS[N_STREAMS] = {'d', 'l'};

const int N_BUFFER = 2;
const char BUFFER[N_BUFFER] = {'d', 'l'};

#endif
