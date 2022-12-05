#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <avr/wdt.h>
#include "TinyMPU6050.h"
#include "RTClib.h"
#include "MAX6675.h"
#define WSS_PIN 3       //pin do przerwan z WSS
#define ERR_PIN 4       //pin z dioda sygnalizacji bledu
#define CS_PIN_TC 9     //pin wyboru termopary dla SPI
#define CS_PIN_SD 10    //pin wyboru karty SD dla SPI
#define TPS_PIN A0      //pin do pomiaru polozenia przepustnicy
#define CHARGE_PIN A1   //pin do pomiaru ladowania akumulatora

//DATALOGGER:
RTC_DS1307 rtc;
DateTime now;
File logFile;
String dataRecord = "";
String fileName = "";
unsigned int idx = 0;
bool triggerDataCollect = false;
//WHEEL SPEED SENSOR:
unsigned int RPM = 0;
unsigned long spinStart = micros();
unsigned long spinDuration;
unsigned int resetCounter = 0;
//TERMOPARA:
MAX6675 thermocouple(CS_PIN_TC);
float celsius = 0;
//AKCELEROMETR:
MPU6050 accelerometer(Wire);
//POMIARY NAPIECIA:
int TPS;
float chargeVoltage;

void wheelSpin() {
    spinDuration = micros() - spinStart;
    spinStart = micros();
    RPM = 60000000 / spinDuration;
    resetCounter = 0;
}

void dateTime (uint16_t* date, uint16_t* time) {
    *date = FAT_DATE(now.year(), now.month(), now.day());
    *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void setup() {
    Serial.begin(9600);
    pinMode(WSS_PIN, INPUT);
    pinMode(ERR_PIN, OUTPUT);
    pinMode(CS_PIN_TC, OUTPUT);
    pinMode(CS_PIN_SD, OUTPUT);
    pinMode(TPS_PIN, INPUT);
    pinMode(CHARGE_PIN, INPUT);
    digitalWrite(ERR_PIN, HIGH);
    digitalWrite(CS_PIN_SD, HIGH);
    digitalWrite(CS_PIN_TC, HIGH);
    attachInterrupt(digitalPinToInterrupt(3), wheelSpin, FALLING);

    //KONFIGURACJA RTC I KARTY SD
    #ifndef ESP8266
        while (!Serial); // wait for serial port to connect. Needed for native USB
    #endif

    if (!rtc.begin()) {
        Serial.println("Couldn't find RTC");
        Serial.flush();
        while (1) delay(10);
    }

    if (!rtc.isrunning()) {
        Serial.println("RTC is NOT running, let's set the time!");
        // When time needs to be set on a new device, or after a power loss, the
        // following line sets the RTC to the date & time this sketch was compiled
        rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    }

    Serial.print("Initializing SD card...");

    // see if the card is present and can be initialized:
    if (!SD.begin(CS_PIN_SD)) {
        Serial.println("Card failed, or not present");
        // don't do anything more:
        while (1);
    }
    Serial.println("card initialized.");

    //INICJALIZACJA PLIKU CSV
    now = rtc.now();
    SdFile::dateTimeCallback(dateTime);
    fileName += String(now.day());
    fileName += String(now.hour());
    fileName += String(now.minute());
    fileName += String(now.second());
    fileName += ".csv";
    Serial.println(fileName);
    logFile = SD.open(fileName, FILE_WRITE);
    if (logFile) {
        logFile.println("index,time,temperature,accel X,accel Y,accel Z,RPM,TPS,voltage");
        //logFile.close();
    }
    else {
        Serial.println("blad otwierania pliku");
    }

    //INICJALIZACJA I KALIBRACJA AKCELEROMETRU
    accelerometer.Initialize();
    Serial.println("=====================================");
    Serial.println("Accelerometer: Starting calibration...");
    accelerometer.Calibrate();
    Serial.println("Accelerometer: Calibration complete!");

    //KONFIGURACJA PRZERWAN
    cli();                                  //clear interrupt - wylacz wszystkie przerwania
    WDTCSR = (1 << WDCE);                   //Watchdog change enable - umozliwia zmiane prescalera
    WDTCSR = (1 << WDIE);                   //Watchdog interrupt enable
    WDTCSR = (1 << WDP1) | (1 << WDP0);     //prescaler na 0.125s
    TCCR1A = 0;                             //wyzeruj rejestr generatora PWM na timer1
    TCCR1B = B00000011;                     //prescaler timer1 na 64 (0.262s)
    TIMSK1 |= B00000010;                    //wlacz compare interrupt na rejestrze A timer1 (OCIE1A=1)
    OCR1A = 25000;                          //ustaw wartosc do porownywania dla timer1 (16kHz/64*0.1s = 25000)
    TCNT1 = 0;                              //reset timer1
    TCCR2A = 0;                             //wyzeruj rejestr generatora PWM na timer2
    TCCR2B = B00000111;                     //prescaler timer2 na 1024 (0.016s)
    TIMSK2 |= B00000001;                    //wlacz overflow interrupt na rejestrze A timer1 (TOIE=1)
    TCNT2 = 0;                              //reset timer2
    sei();                                  //set interrupt - wlacz przerwania

    digitalWrite(ERR_PIN, LOW);
}

void loop() {
    if (triggerDataCollect) {
        wdt_reset();
        triggerDataCollect = false;
        now = rtc.now();                                //zapisz godzine
        celsius = thermocouple.readTempC();             //odczytaj temperature
        accelerometer.Execute();                        //odczytaj akcelerometr
        TPS = (analogRead(TPS_PIN) - 125) / 6.03;       //odczytaj polozenie przepustnicy
        chargeVoltage = analogRead(CHARGE_PIN) / 68.2;  //odczytaj ladowanie akumulatora


        //numerowanie linii
        dataRecord = String(idx++);
        dataRecord += ',';
        //zamien timestamp na string
        dataRecord += String(now.hour());
        dataRecord += ':';
        dataRecord += String(now.minute());
        dataRecord += ':';
        dataRecord += String(now.second());
        dataRecord += ',';
        //zamien temperature na string
        dataRecord += String(celsius);
        dataRecord += ',';
        //zapisz przyspieszenie
        dataRecord += String(accelerometer.GetAccX());
        dataRecord += ',';
        dataRecord += String(accelerometer.GetAccY());
        dataRecord += ',';
        dataRecord += String(accelerometer.GetAccZ());
        dataRecord += ',';
        //zapisz obroty kola
        dataRecord += String(RPM);
        dataRecord += ',';
        //zapisz polozenie przepustnicy
        dataRecord += String(TPS);
        dataRecord += "%,";
        //zapisz napiecie ladowania
        dataRecord += String(chargeVoltage);

        //zapisz linie danych
        if (logFile) {
            logFile.println(dataRecord);
            logFile.flush();
            //logFile.close();
            Serial.println(dataRecord);
        }
        else {
            Serial.println("blad otwierania pliku");
            digitalWrite(ERR_PIN, HIGH);
        }
    }
}

ISR(WDT_vect) {
    digitalWrite(ERR_PIN, HIGH);
}

ISR(TIMER1_COMPA_vect) {
    TCNT1 = 0;      //reset timer1
    triggerDataCollect = true;
}

ISR(TIMER2_OVF_vect) {
    if (++resetCounter >= 63) RPM = 0;      //po sekundzie bezczynnosci wyzeruj RPM
}