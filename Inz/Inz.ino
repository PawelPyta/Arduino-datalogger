#include <microDS18B20.h>
#include "TinyMPU6050.h"
#include <RTClib.h>
#include <SD.h>
#include <avr/wdt.h>
#define TEM_PIN 2       //pin do komunikacji z czujnikiem temp.
#define WSS_PIN 3       //pin do przerwan z WSS
#define ERR_PIN 4       //pin z dioda sygnalizacji bledu
#define TEMP_LED_WSS 5  //dioda testowa WSS
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
bool led = false;
//CZUJNIK TEMPERATURY:
MicroDS18B20<TEM_PIN> temperatureSensor;
float tempCelsius = 0;
//AKCELEROMETR:
MPU6050 accelerometer(Wire);
float x = 0; float y = 0; float z = 0;      //wartosci sredniej kroczacej
float x1 = 0; float y1 = 0; float z1 = 0;   //odczyty historyczne
float x2 = 0; float y2 = 0; float z2 = 0;
float x3 = 0; float y3 = 0; float z3 = 0;
float x4 = 0; float y4 = 0; float z4 = 0;
float x5 = 0; float y5 = 0; float z5 = 0;
//POMIARY NAPIECIA:
int TPS;
float chargeVoltage;

void wheelSpin() {
    spinDuration = micros() - spinStart;
    //Sprawdzanie, czy szybkosc obrotu kola jest "rozsadna", tj. nie wystapil falszywy impuls
    if (spinDuration > 60000) {
        spinStart = micros();
        RPM = 60000000 / spinDuration;
        resetCounter = 0;
        led = !led;
        if (led)
            digitalWrite(TEMP_LED_WSS, HIGH);
        else
            digitalWrite(TEMP_LED_WSS, LOW);
    }
}

void dateTime (uint16_t* date, uint16_t* time) {
    *date = FAT_DATE(now.year(), now.month(), now.day());
    *time = FAT_TIME(now.hour(), now.minute(), now.second());
}

void signalError(bool yes = true) {
    if (yes) digitalWrite(ERR_PIN, HIGH);
    else digitalWrite(ERR_PIN, LOW);
}

void setup() {
    Serial.begin(9600);
    pinMode(WSS_PIN, INPUT);
    pinMode(ERR_PIN, OUTPUT);
    pinMode(TEMP_LED_WSS, OUTPUT);
    pinMode(CS_PIN_SD, OUTPUT);
    pinMode(TPS_PIN, INPUT);
    pinMode(CHARGE_PIN, INPUT);
    digitalWrite(ERR_PIN, HIGH);
    digitalWrite(TEMP_LED_WSS, LOW);
    digitalWrite(CS_PIN_SD, HIGH);
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
        Serial.flush();
        // don't do anything more:
        while (1) delay(10);
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
        Serial.flush();
        while (1) delay(10);
    }

    //INICJALIZACJA I KALIBRACJA AKCELEROMETRU
    accelerometer.Initialize();
    Serial.println("=====================================");
    Serial.println("Accelerometer: Starting calibration...");
    accelerometer.Calibrate();
    Serial.println("Accelerometer: Calibration complete!");

    //KONFIGURACJA CZUJNIKA TEMPERATURY:
    temperatureSensor.setResolution(9);

    //KONFIGURACJA PRZERWAN
    cli();                                  //clear interrupt - wylacz wszystkie przerwania
    WDTCSR = (1 << WDCE);                   //Watchdog change enable - umozliwia zmiane prescalera
    WDTCSR = (0 << WDE);                    //Watchdog system reset disable
    WDTCSR = (1 << WDIE);                   //Watchdog interrupt enable
    WDTCSR = (1 << WDP1) | (1 << WDP0);     //prescaler na 0.125s
    //TCCR1A = 0;                             //wyzeruj rejestr generatora PWM na timer1
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

/*
    G³ówna pêtla programu sprawdza stan triggera wyzwalaj¹cego akwizycjê danych z czujników.
    Je¿eli trigger ma wartoœæ true, pobierane s¹ odczyty, nastêpnie przeliczane, formatowane do stringa
    i zapisywane na karcie SD. Po przejœciu tej procedury trigger jest ustawiany na false.
    Trigger wyzwalaj¹cy jest aktywowany przez przerwanie z timera co dok³adnie 100ms.
*/
void loop() {
    if (triggerDataCollect) {
        wdt_reset();                        //zresetuj licznik watchdoga
        triggerDataCollect = false;

        if (rtc.isrunning()) {
            now = rtc.now();                            //zapisz godzine
            signalError(false);
        } else 
            signalError(true);

        if (temperatureSensor.readTemp()) {
            temperatureSensor.requestTemp();
            tempCelsius = temperatureSensor.getTemp();  //odczytaj temperature
            signalError(false);
        } else
            signalError(true);

        accelerometer.Execute();                        //odczytaj akcelerometr
        //x5 = x4; y5 = y4; z5 = z4;                      //obliczanie sredniej kroczacej
        //x4 = x3; y4 = y3; z4 = z3;
        //x3 = x2; y3 = y2; z3 = z2;
        //x2 = x1; y2 = y1; z2 = z1;
        //x = accelerometer.GetAccX();/*
        //y = accelerometer.GetAccY();
        //z = accelerometer.GetAccZ();*/
        /*x = (x1 + x2 + x3 + x4 + x5) / 5;
        y = (y1 + y2 + y3 + y4 + y5) / 5;
        z = (z1 + z2 + z3 + z4 + z5) / 5;*/
        TPS = (analogRead(TPS_PIN) - 125) / 6.03;       //odczytaj polozenie przepustnicy
        chargeVoltage = analogRead(CHARGE_PIN) / 51.54; //odczytaj ladowanie akumulatora


        //numerowanie linii
        dataRecord = String(idx++);
        dataRecord += ',';
        //zamien timestamp na string
        dataRecord += now.timestamp(DateTime::TIMESTAMP_TIME);
        dataRecord += ',';
        //zamien temperature na string
        dataRecord += String(tempCelsius);
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
        if (logFile) {                      //sprawdzanie, czy plik jest otwarty
            logFile.println(dataRecord);
            logFile.flush();
            Serial.println(dataRecord);
            signalError(false);
        }
        else {
            Serial.println("blad otwierania pliku");
            signalError(true);              //jezeli wystapi problem z plikiem, zasygnalizuj blad
        }
    }
}

ISR(WDT_vect) {
    digitalWrite(ERR_PIN, HIGH);            //sygnalizacja bledu dioda LED
}

ISR(TIMER1_COMPA_vect) {
    TCNT1 = 0;                              //reset timer1
    triggerDataCollect = true;              //wyzwalanie procedury akwizycji danych
}

ISR(TIMER2_OVF_vect) {
    if (++resetCounter >= 126) RPM = 0;     //po 2 sekundach bezczynnosci wyzeruj RPM
}