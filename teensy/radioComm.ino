//Arduino/Teensy Flight Controller - dRehmFlight
//Author: Nicholas Rehm
//Project Start: 1/6/2020
//Last Updated: 7/29/2022
//Version: Beta 1.3

static unsigned long rising_edge_start_1, rising_edge_start_2, rising_edge_start_3, rising_edge_start_4, rising_edge_start_5, rising_edge_start_6; 
static unsigned long channel_1_raw, channel_2_raw, channel_3_raw, channel_4_raw, channel_5_raw, channel_6_raw;

void radioSetup()
{
    Serial1.begin(115000);
}

void serialEvent1(void)
{
    while (Serial1.available()) {
        DSM.handleSerialEvent(Serial1.read(), micros());
    }
}


void getCh1() {
    int trigger = digitalRead(ch1Pin);
    if(trigger == 1) {
        rising_edge_start_1 = micros();
    }
    else if(trigger == 0) {
        channel_1_raw = micros() - rising_edge_start_1;
    }
}

void getCh2() {
    int trigger = digitalRead(ch2Pin);
    if(trigger == 1) {
        rising_edge_start_2 = micros();
    }
    else if(trigger == 0) {
        channel_2_raw = micros() - rising_edge_start_2;
    }
}

void getCh3() {
    int trigger = digitalRead(ch3Pin);
    if(trigger == 1) {
        rising_edge_start_3 = micros();
    }
    else if(trigger == 0) {
        channel_3_raw = micros() - rising_edge_start_3;
    }
}

void getCh4() {
    int trigger = digitalRead(ch4Pin);
    if(trigger == 1) {
        rising_edge_start_4 = micros();
    }
    else if(trigger == 0) {
        channel_4_raw = micros() - rising_edge_start_4;
    }
}

void getCh5() {
    int trigger = digitalRead(ch5Pin);
    if(trigger == 1) {
        rising_edge_start_5 = micros();
    }
    else if(trigger == 0) {
        channel_5_raw = micros() - rising_edge_start_5;
    }
}

void getCh6() {
    int trigger = digitalRead(ch6Pin);
    if(trigger == 1) {
        rising_edge_start_6 = micros();
    }
    else if(trigger == 0) {
        channel_6_raw = micros() - rising_edge_start_6;
    }
}
