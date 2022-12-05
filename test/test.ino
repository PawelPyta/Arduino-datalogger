#include "avr/interrupt.h"

unsigned int count = 0;

void funk() {
    Serial.println(count);
    count++;
}

void setup() {
    Serial.begin(9600);
    cli();                                  //clear interrupt - wylacz wszystkie przerwania
    TCCR1A = 0;
    //TCCR1B = 0;
    TCCR1B = B00000011;                    //prescaler timer1 na 64 (0.262s)
    TIMSK1 |= B00000010;                    //wlacz compare interrupt na rejestrze A (OCIE1A=1)
    OCR1A = 25000;                          //ustaw wartosc do porownywania (16kHz/64*0.1s = 25000)
    TCNT1 = 0;                              //reset timer1
    sei();                                  //set interrupt - wlacz przerwania
}

void loop() {
}

ISR(TIMER1_COMPA_vect) {
    TCNT1 = 0;      //reset timer1
    funk();
}