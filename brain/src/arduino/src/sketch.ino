/**
 * (C) Copyright 2012, Assistobot
 */

#include <stdio.h>

unsigned long utime_last = 0;
enum {SHT11_ACK, SHT11_DATA} sht11_state = SHT11_ACK;
unsigned char sht11_bit_idx = 0;
unsigned char sht11_data_list[5];
unsigned char sht11_ready = 0;

void setup() {
    Serial.begin(9600);

    pinMode(2, INPUT);
    attachInterrupt(0, sht11_read_callback, FALLING);
}

void loop() {
    //TODO: Remove delays:
    pinMode(2, OUTPUT);
    digitalWrite(2, HIGH);
    delay(1);
    digitalWrite(2, LOW);
    delay(25);
    digitalWrite(2, HIGH);
    pinMode(2, INPUT);
    delay(1000);

    if (sht11_ready) {
        char str_buf[255];
        sprintf(str_buf, "hum=%d.%d,temp=%d.%d\n", 
                sht11_data_list[0], sht11_data_list[1],
                sht11_data_list[2], sht11_data_list[3]);
        Serial.write(str_buf);
        sht11_ready = 0;
    }
}


void sht11_read_callback() {
    //TODO:XXX: Account for overflow after ~70 Minutes:
    unsigned long utime_cur = micros();
    unsigned long utime_diff = utime_cur - utime_last;
    utime_last = utime_cur;

    //Reset if the delay is too long or the ready flag has not been cleared:
    if (utime_diff > 200 || sht11_ready == 1) {
        sht11_state = SHT11_ACK; 
        return;
    }

    unsigned char bit_value = utime_diff > 100;

    switch (sht11_state) {
        case SHT11_ACK:
            //Start of transmission:
            sht11_state = SHT11_DATA;
            sht11_data_list[0] = 0;
            sht11_data_list[1] = 0;
            sht11_data_list[2] = 0;
            sht11_data_list[3] = 0;
            sht11_data_list[4] = 0;
            sht11_bit_idx = 0;
            break;
        case SHT11_DATA:
            //Proces a single bit:
            sht11_data_list[sht11_bit_idx / 8] |= bit_value << 7 - (sht11_bit_idx % 8);
            sht11_bit_idx++;
            if (sht11_bit_idx == 40) {
                //Ok, all data has been received:
                sht11_ready = 1;
                sht11_state = SHT11_ACK;
            }
            break;
    }
}
