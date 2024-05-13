#include <LiquidCrystal.h> // for lcd display
#include <Stepper.h> // for stepper motor
#include <RTClib.h> // for real time clock
#include <Wire.h> // for real time clock
#include <DHT.h> 

#define TBE 0x20 

volatile unsigned char* my_UCSR0A = (unsigned char*)0x00C0;
volatile unsigned char* my_UCSR0B = (unsigned char*)0x00C1;
volatile unsigned char* my_UCSR0C = (unsigned char*)0x00C2;
volatile unsigned int* my_UBRR0 = (unsigned int*)0x00C4;
volatile unsigned char* my_UDR0 = (unsigned char*)0x00C6;


volatile unsigned char* my_ADMUX = (unsigned char*)0x7C;
volatile unsigned char* my_ADCSRA = (unsigned char*)0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*)0x78;


volatile unsigned char* my_EIMSK = (unsigned char*)0x3D;
volatile unsigned char* my_EICRA = (unsigned char*)0x69;


volatile unsigned char* DDR_A = (unsigned char*)0x21;
volatile unsigned char* PORT_A = (unsigned char*)0x22;
volatile unsigned char* PIN_A = (unsigned char*)0x20;


volatile unsigned char* DDR_B = (unsigned char*)0x24;
volatile unsigned char* PIN_B = (unsigned char*)0x23;


volatile unsigned char* DDR_C = (unsigned char*)0x27;
volatile unsigned char* PORT_C = (unsigned char*)0x28;
volatile unsigned char* PIN_C = (unsigned char*)0x26;

volatile unsigned char* DDR_D = (unsigned char*)0x2A;
volatile unsigned char* PORT_D = (unsigned char*)0x2B;
volatile unsigned char* PIN_D = (unsigned char*)0x29;


volatile unsigned char* DDR_E = (unsigned char*)0x2D;
volatile unsigned char* PORT_E = (unsigned char*)0x2E;


volatile unsigned char* my_TCCR1A = (unsigned char*)0x80;
volatile unsigned char* my_TCCR1B = (unsigned char*)0x81;
volatile unsigned char* my_TIMSK1 = (unsigned char*)0x6F;
volatile unsigned int* my_TCNT1 = (unsigned  int*)0x84;

volatile unsigned int timer_counter = 1;

// creating an lcd object
// RS at digital pin 7, enable at digital pin 8, D4 at digital pin 9, D5 at digital pin 10, D6 at digital pin 11, D7 at digital pin 12
LiquidCrystal lcd(7, 8, 9, 10, 11, 12);

// creating stepper object, 64 steps per revolution, digital pins 3 through 6
Stepper myStepper(64, 6, 4, 5, 3);

// creating real time clock object
RTC_DS1307 rtc;

// creating dht object for humidity and temp
DHT DHT(A10, DHT11);


// chose threshold 285 - very unstable readings with sensor
const int waterLevelThreshold = 275;

// enablePin for fan motor using L293D
const int enablePin = 2;

// max speed 20rpm for 28BYJ-48 motor at 5V
const unsigned int motorSpeed = 20;

// fan running or not
bool running = false;

//vent direction
bool changeDirection = true;

// in error state or not
bool errorState = false;
bool noError = true;

// variable to track current state
int currentState = 0;

// variable to track last state
int lastState = -1;

// temp threshold for turning on fan
const int tempLimit = 0;

// constant for printing temp/humidity on display every minute
const unsigned int oneMinute = 60000;

// variable to print temp/humidity to LCD at startup or if we just cleared error message from LCD
bool startFlag = true;

// variable for start button pressed
bool startPressed = false;



void set_interrupt_regs();
void adc_init();
void controlVent();
void state();
void stopButton();
void monitorWater();
void tempHumidity();
void fanOperation(int caller);
void eventTimeReport(int caller);
void toChar(unsigned int value, char* array);
void printTimeAndDate(unsigned int length, char* array);
void set_interrupt_regs();
unsigned int adc_read(unsigned char adc_channel_num);
void U0init(int U0baud);
void U0putchar(unsigned char U0pdata);

void setup()
{

    U0init(9600);
    adc_init();
    myStepper.setSpeed(motorSpeed);
    rtc.begin();
    rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
    Wire.begin();
    lcd.begin(16, 2);

    // fan motor using L293D
    *DDR_A |= (1 << 0); // input2, OUTPUT (PA0)
    *DDR_A |= (1 << 2); // input1, OUTPUT (PA2)
    *DDR_E |= (1 << 4); // enablePin, OUTPUT (PE4)

    // buttons reset and stop
    *DDR_C &= ~(1 << 7); // reset button, INPUT (PC7)
    *DDR_A &= ~(1 << 3); // stop button, INPUT (PA3)

    // LED's
    *DDR_C |= (1 << 3); // yellowLED, OUTPUT (PC3)
    *DDR_C |= (1 << 1); // greenLED, OUTPUT (PC1)
    *DDR_D |= (1 << 7); // redLED, OUTPUT (PD7)
    *DDR_A |= (1 << 1); // blueLED, OUTPUT (PA1)

    // button to control vent direction, INPUT (PB1)
    *DDR_B &= ~(1 << 1);
    set_interrupt_regs();
}

void loop()
{
    state();
}

void state()
{
    switch (currentState)
    {
    case 0: // disabled
        if (currentState != lastState)
        {
            lcd.clear();
            // print state disabled with time stamp
            eventTimeReport(0);

            // turn yellow LED 
            *PORT_C |= (1 << 3); // yellowLED
            *PORT_C &= ~(1 << 1); // greenLED
            *PORT_D &= ~(1 << 7); // redLED
            *PORT_A &= ~(1 << 1); // blueLED
            lastState = currentState;
            *my_EIMSK |= (1 << 3);
        }
        else
        {
            controlVent();

            if (startPressed)
            {
                startPressed = false;
                currentState = 1;
            }
        }
        break;

    case 1: 
        if (currentState != lastState)
        {
            lcd.setCursor(0, 1);
            lcd.print("                ");


            *PORT_C &= ~(1 << 3); 
            *PORT_C |= (1 << 1); 
            *PORT_D &= ~(1 << 7); 
            *PORT_A &= ~(1 << 1); 


            eventTimeReport(4);
            lastState = currentState;
        }
        else
        {

            noError = true;
            monitorWater();
            controlVent();
            tempHumidity();
            if (*PIN_A & (1 << 3))
            {
                stopButton();
            }
        }
        break;

    case 2: // error
        if (currentState != lastState)
        {
            // in error state now
            noError= false;
            *PORT_C &= ~(1 << 3); // yellowLED
            *PORT_C &= ~(1 << 1); // greenLED
            *PORT_D |= (1 << 7); // redLED
            *PORT_A &= ~(1 << 1); // blueLED
            if (running)
            {
                fanOperation(2);
            }
            eventTimeReport(5);
            lastState = currentState;
        }
        else
        {
            unsigned int waterLevel = adc_read(0);
            if (waterLevel < waterLevelThreshold)
            {
                tempHumidity();
                errorState = true;
                lcd.setCursor(0, 1);
                lcd.print("Error Low Water");

                if (*PIN_A & (1 << 3))
                {
                    // reset timer and send to disabled state
                    stopButton();
                }
            }

            else if (waterLevel > waterLevelThreshold)
            {
                // monitor temp and humidity
                tempHumidity();

                if (errorState)
                {
                    lcd.setCursor(0, 1);
                    lcd.print("Level Ok  Reset");
                    // set error state to false
                    errorState = false;
                }

                else if (*PIN_C & (1 << 7))
                {
                    currentState = 1;
                }

                else if (*PIN_A & (1 << 3))
                {
                    stopButton();
                }
            }
        }
        break;

    case 3: // running
        if (currentState != lastState)
        {
            // set blue LED and clear the rest
            *PORT_C &= ~(1 << 3); // yellowLED
            *PORT_C &= ~(1 << 1); // greenLED
            *PORT_D &= ~(1 << 7); // redLED
            *PORT_A |= (1 << 1); // blueLED
            eventTimeReport(6);

            // variable to only do this once
            lastState = currentState;
        }
        else
        {
            monitorWater();
            controlVent();
            tempHumidity();
            if (*PIN_A & (1 << 3))
            {
                
                fanOperation(2);
                stopButton();
            }
        }
        break;
    }
}

void stopButton()
{

    *my_TIMSK1 &= ~(1 << 0);
    timer_counter = 1;
    *my_TCNT1 = 49910;
    startFlag = true;
    currentState = 0;
}

void monitorWater()
{
    unsigned int waterLevel = adc_read(0);
    if (waterLevel < waterLevelThreshold)
    {
        currentState = 2; // error
    }
}

void controlVent()
{
    if (*PIN_B & (1 << 1) && changeDirection == true)
    {
        // shift vent direction 180 degrees
        myStepper.step(1024);
        // variable to make sure we are only shifting 180 degrees
        changeDirection = false;
        // print change of position 
        eventTimeReport(1);
    }

    else if (*PIN_B & (1 << 1) && changeDirection == false)
    {
        myStepper.step(-1024);
        changeDirection = true;
        eventTimeReport(1);
    }
}


void tempHumidity()
{
    // read in the temperature and humidity
    int value = DHT.read(A10);
    int temp = DHT.readTemperature(false,false);
    int humidity = DHT.readHumidity(false);

    if (startFlag)
    {
        startFlag = false;
        lcd.setCursor(0, 0);
        lcd.print("T: ");
        lcd.print(temp);
        lcd.print("degC");
        lcd.print(" H: ");
        lcd.print(humidity);
        lcd.print("%");
        *my_TIMSK1 |= (1 << 0);

    }

    if (noError)
    {
        if (temp >= tempLimit && running == false)
        {
            fanOperation(1);
            currentState = 3;
        }

        else if (temp < tempLimit && running == true)
        {
            fanOperation(2);
            currentState = 1;
        }
    }
}

void fanOperation(int input)
{
    
    if (input == 1)
    {
        running = true;
        *PORT_E |= (1 << 4);
        *PORT_A |= (1 << 2);
        *PORT_A &= ~(1 << 0);
        eventTimeReport(2);
    }

    else if (input == 2)
    {
        running = false;
        *PORT_E &= ~(1 << 4);
        *PORT_A &= ~(1 << 2);
        *PORT_A &= ~(1 << 0);
        eventTimeReport(3);
    }
}

void eventTimeReport(int input)
{
    if (input== 0)
    {
        printMessage("State: DISABLED", 15);
    }
    else if (input== 1)
    {
        printMessage("Stepper motor position for the vent moved.", 42);
    }
    else if (input == 2)
    {
        printMessage("Fan started.", 12);
    }
    else if (input == 3)
    {
        printMessage("Fan stopped.", 12);
    }
    else if (input == 4)
    {
        printMessage("State: IDLE", 11);
    }
    else if (input == 5)
    {
        printMessage("State: ERROR", 12);
    }
    else if (input == 6)
    {
        printMessage("State: RUNNING", 14);
    }

    DateTime now = rtc.now();
    unsigned int hr = now.hour();
    unsigned int min = now.minute();
    unsigned int sec = now.second();
    unsigned int d = now.day();
    unsigned int mnth = now.month();
    unsigned int yr = now.year();


    char hour[2];
    char minute[2];
    char second[2];
    char month[2];
    char day[2];


    toChar(hr, hour);
    toChar(min, minute);
    toChar(sec, second);
    toChar(mnth, month);
    toChar(d, day);


    char year[4];
    for (int i = 3; i >= 0; i--)
    {
        year[i] = yr % 10 + '0';
        yr /= 10;
    }
    char textOne[7] = "Date: ";
    char textTwo[9] = ", Time: ";

    printTimeAndDate(6, textOne);
    printTimeAndDate(2, month);
    U0putchar('/');
    printTimeAndDate(2, day);
    U0putchar('/');
    printTimeAndDate(4, year);
    printTimeAndDate(8, textTwo);
    printTimeAndDate(2, hour);
    U0putchar(':');
    printTimeAndDate(2, minute);
    U0putchar(':');
    printTimeAndDate(2, second);
    U0putchar(0x0A);
}

void printMessage(const String& message, const int& length)
{
    for (int i = 0; i < length; i++)
    {
        U0putchar(message[i]);
    }
    U0putchar(0x0A);
}

void toChar(unsigned int value, char* array)
{
    if (value < 10)
    {
        array[0] = '0';
        array[1] = value + '0';
    }

    else
    {
        array[1] = value % 10 + '0';
        value /= 10;
        array[0] = value % 10 + '0';
    }
}

void printTimeAndDate(unsigned int length, char* array)
{
    for (int i = 0; i < length; i++)
    {
        U0putchar(array[i]);
    }
}

ISR(INT3_vect)
{
    if (*PIN_D & (1 << 3))
    {
        startPressed = true;
        *my_EIMSK &= ~(1 << 3);
    }
}

ISR(TIMER1_OVF_vect)
{
    if (timer_counter >= 60)
    {
        startFlag = true;
        timer_counter = 1;
    }
    timer_counter++;
    *my_TCNT1 = 49910; // *my_TCNT1 = 65535 – (16*10^6  * 1 / 1024)
}

void set_interrupt_regs()
{
    *DDR_D &= ~(1 << 3); // start button, INPUT (PD3)
    *PORT_D |= (1 << 3);; // pull up resistor for start button
    *my_EICRA |= (1 << 6); // ISC30
    *my_EICRA |= (1 << 7); // ISC31
    *my_TCCR1A = 0;
    *my_TCCR1B = 0;
    *my_TCNT1 = 49910;
    *my_TCCR1B |= (1 << 0); // CS10
    *my_TCCR1B |= (1 << 2); // CS12
}
void adc_init()
{
    // setup the A register
    *my_ADCSRA |= 0b10000000; // set bit   7 to 1 to enable the ADC
    *my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
    *my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
    *my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
    // setup the MUX Register
    *my_ADMUX &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
    *my_ADMUX |= 0b01000000; // set bit   6 to 1 for AVCC analog reference
    *my_ADMUX &= 0b11011111; // clear bit 5 to 0 for right adjust result
    *my_ADMUX &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num)
{
    *my_ADMUX &= 0b11100000;
    *my_ADMUX += adc_channel_num;
    *my_ADCSRA |= 0x40;
    while ((*my_ADCSRA & 0x40) != 0);
    return *my_ADC_DATA;
}

void U0init(int U0baud)
{
    unsigned long FCPU = 16000000;
    unsigned int tbaud;
    tbaud = (FCPU / 16 / U0baud - 1);
    // Same as (FCPU / (16 * U0baud)) - 1;
    *my_UCSR0A = 0x20;
    *my_UCSR0B = 0x18;
    *my_UCSR0C = 0x06;
    *my_UBRR0 = tbaud;
}

void U0putchar(unsigned char U0pdata)
{
    while ((*my_UCSR0A & TBE) == 0);
    *my_UDR0 = U0pdata;
}