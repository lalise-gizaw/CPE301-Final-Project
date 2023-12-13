/****************************************
 * Names: Vanessa Kalenits, Lalise Gizaw
 * Assignment: Lab Final
 * Date: December 12, 2023
 ****************************************/
//worked on code together in person on Arduino

// Necessary libraries needed 
#include "Adafruit_Sensor.h" // Adafruit sesonsor by 
#include "DHT.h" // DHT sensor byy Adafruit
#include <LiquidCrystal.h> // By arduino, adafruit
#include <Stepper.h> // for stepper 
#include <RTClib.h> // RTTClib.h by adafruit 

//DHT TEMP, HUMIDITY
#define dht_p 35 // define dht pin at 35
#define DHTTYPE DHT11
DHT dht(dht_p, DHTTYPE);

//LCD
//#include <LiquidCrystal.h>
//const int rs = 7, en = 6, d4 = 5, d5 = 4, d6 = 3, d7 = 2;
LiquidCrystal lcd(7, 6, 5, 4, 3, 2);


//STEPPER
#define steps 32
Stepper stepper(steps, 37, 41, 39, 43);


//CLOCK
//#include <RTClib.h>
RTC_DS1307 rtc;

//UART
#define RDA 0x80
#define TBE 0x20


//diplay terminal 
volatile unsigned char *myUCSR0A = (unsigned char *)0x00C0;
volatile unsigned char *myUCSR0B = (unsigned char *)0x00C1;
volatile unsigned char *myUCSR0C = (unsigned char *)0x00C2;
volatile unsigned int *myUBRR0 = (unsigned int *) 0x00C4;
volatile unsigned char *myUDR0 = (unsigned char *)0x00C6;


//define the port register for the LED lights 
volatile unsigned char* port_a = (unsigned char*) 0x22;
volatile unsigned char* ddr_a = (unsigned char*) 0x21;
volatile unsigned char* pin_a = (unsigned char*) 0x20;

//For analog
volatile unsigned char* my_ADMUX = (unsigned char*) 0x7C;
volatile unsigned char* my_ADCSRB = (unsigned char*) 0x7B;
volatile unsigned char* my_ADCSRA = (unsigned char*) 0x7A;
volatile unsigned int* my_ADC_DATA = (unsigned int*) 0x78;

//my_delay Timer Pointers
volatile unsigned char *myTCCR1A = (unsigned char *) 0x80;
volatile unsigned char *myTCCR1B = (unsigned char *) 0x81;
volatile unsigned char *myTCCR1C = (unsigned char *) 0x82;
volatile unsigned char *myTIMSK1 = (unsigned char *) 0x6F;
volatile unsigned int *myTCNT1 = (unsigned int *) 0x84;
volatile unsigned char *myTIFR1 = (unsigned char *) 0x36;
volatile unsigned char *portDDRB = (unsigned char *) 0x24;
volatile unsigned char *portB = (unsigned char *) 0x25;

//define port register for Analog
volatile unsigned char* port_f = (unsigned char*) 0x31;
volatile unsigned char* ddr_f = (unsigned char*) 0x30;
volatile unsigned char* pin_f = (unsigned char*) 0x2F;


//threshold
float threshTemp = 50; // We used this threshold when we wanted a running state
//float treshTemp = 100; // We used this threshold line when testing for Idle state 
float threshwater = 35; // threshold for water 

//interupt buttons
const byte stopbutton_p = 18; // stop button pin
const byte resetbutton_p = 19; // reset button pin

volatile bool state;
int timer= 0; //global ticks counter

// Function definition 
//States
void Disabled_S();
void IDLE_S();
void Running_S();
void Error_S();

void Temp_Humdity();
void stop_button();
void reset_button();
void stepper_motor();
void SerialMonitor();


void setup() {
  //Interrupt
  //set PB4 to output
*portDDRB |= 0b11110000;
  //set PB4 LOW
*portB &= 0b11101111;
  //set analog ports to output
*ddr_f |= 0b00000001;

  attachInterrupt(digitalPinToInterrupt(stopbutton_p), stop_button, FALLING);
  attachInterrupt(digitalPinToInterrupt(resetbutton_p), reset_button, FALLING);

  //setup the Timer for Normal Mode, with the TOV interrupt enabled
  setup_timer_regs();
  //Start the UART
  U0init(9600);
  // Serial.begin(9600); for testing
stepper.setSpeed(500);
lcd.begin(16,2);
dht.begin();
  adc_init();

  //CLOCK
  if (! rtc.begin()) {
Serial.println("Couldn't find RTC");
Serial.flush();
    while (1) my_delay(10);
  }

  if (! rtc.isrunning()) {
Serial.println("RTC is not running");
    // if time needs to be set on new device set RTC to date & time sketch was compiled
rtc.adjust(DateTime(F(__DATE__), F(__TIME__)));
   
  }
}


void loop()
{

  //Get current temp and water sensor values 
int water_sv;
water_sv = adc_read(5);
float curr_Temp = dht.readTemperature(true);

  if(water_sv < 100){
    SerialMonitor();
    Error_S();
  }
  else{
    //check if current temp < or equal to threshold 
    // If it is, mak eit running state
      if(curr_Temp >= threshTemp){
          SerialMonitor();
          stepper_motor();
          Running_S();
      }
      // if the current temp is less than treshold, make it idle state
      else{
          stepper_motor();
          SerialMonitor();
          Disabled_S();
      }
  }
// delay a lil
  my_delay(1000);

}



//LCD monitor
void Temp_Humdity(){
float humidity;
float temp;
temp = dht.readHumidity();
temp = dht.readTemperature(true);

  // If the temp or humidty is not being read
  if (isnan(humidity) || isnan(temp)) {
Serial.println(F("Can't read from DHT sensor!"));
    return;
  }
  // else diplay the Tempreture 
lcd.setCursor(0,1); //prints humidity on the second line
lcd.print("Humidity: ");
lcd.print(humidity);
lcd.print("%");

  // And display the Humidity
lcd.clear();
lcd.setCursor(0,0); //prints temperature on first line
lcd.print("Temp: ");
lcd.print(temp);
lcd.print((char) 223);
lcd.print("F");
  
  //delay
  my_delay(1000);
}

//State 
// The following is the function for State 

// Function for the Disabled state
void Disabled_S()
{
  //yellow LED on
*port_a |= 0b00100000;
*port_a &= 0b00100000;
  //set ports for fan off
*portB &= 0b01011111;
  //stopButton();
  //button #2 code to idle
  if(water_sense() > 100){
state = true;
    //idle_state();
lcd.clear();
  }
}

void IDLE_S()
{
  //green LED on
*port_a |= 0b10000000;
*port_a &= 0b10000000;
  //set ports for fan off
*portB &= 0b01011111;
  Temp_Humdity();
}

void Error_S(){
  //red LED on
*port_a |= 0b00001000;
*port_a &= 0b00001000;
  //set ports for fan off
*portB &= 0b01011111;
  //LCD
lcd.clear();
lcd.setCursor(0,0);
lcd.print("Error: ");
lcd.setCursor(0,1);
lcd.print("Water Low");

  if(state == false){
state = true;
  }
}

void Running_S()
{
  //turn on a blue LED
*port_a &= 0b01000010;
*port_a |= 0b01000010;
 
  //set fan on
*portB |= 0b10100000;
  Temp_Humdity();
}

// For the buttons
void stop_button(){
  if(digitalRead(stopbutton_p) == HIGH){
    Disabled_S();
  }
  
}
void reset_button(){
  if(digitalRead(resetbutton_p) == HIGH){
    IDLE_S();
  }
}
//Functioin for stepper 
int prev_dir = 0;
void stepper_motor(){
  //Stepper motor
int curr_dir = adc_read(0); //change direction of the stepper
  SerialMonitor();
stepper.step(curr_dir - prev_dir);
prev_dir = curr_dir;
}

//RTC to serial monitor
void SerialMonitor(){
DateTime now = rtc.now();
char timeStr[9];// 8 characters + null terminator
  sprintf(timeStr, "%02d:%02d:%02d", now.hour(), now.minute(), now.second());
  for(int i = 0; i < 9; i++){
    U0putChar(timeStr[i]);
  }
  U0putChar('\n');

  my_delay(1000);
}

//check water level
unsigned int water_sense() {
int sensor;
sensor = adc_read(5);
  my_delay(1000);
  return sensor;
}

//my_delay
void my_delay(unsigned int freq){
double period = 1.0/double(freq);
double half_period = period/ 2.0f;
double clk_period = 0.0000000625;
unsigned int ticks = half_period / clk_period;
*myTCCR1B &= 0xF8;
*myTCNT1 = (unsigned int) (65536 - ticks);
* myTCCR1B |= 0b00000001;
  while((*myTIFR1 & 0x01)==0);
*myTCCR1B &= 0xF8;
*myTIFR1 |= 0x01;
}

//Timer setup function
void setup_timer_regs(){
  //setup the timer control registers
*myTCCR1A= 0x00;
*myTCCR1B= 0X00;
*myTCCR1C= 0x00;
  //reset the TOV flag
*myTIFR1 |= 0x01;
  //enable the TOV interrupt
*myTIMSK1 |= 0x01;
}

//TIMER OVERFLOW ISR
ISR(TIMER1_OVF_vect){
//Stop the Timer
*myTCCR1B &= 0xF8;
  //Load the Count
  //*myTCNT1 = (unsigned int) (65535 - (unsigned long) (currentTicks));
  //Start the Timer
*myTCCR1B |= 0b00000001;
  //if it's not the STOP amount
  //if(currentTicks != 65535){
    //XOR to toggle PB6
    //*portB ^= 0x40;
  //}
}

void adc_init(){
  // setup the A register
*my_ADCSRA |= 0b10000000; // set bit 7 to 1 to enable the ADC
*my_ADCSRA &= 0b11011111; // clear bit 6 to 0 to disable the ADC trigger mode
*my_ADCSRA &= 0b11110111; // clear bit 5 to 0 to disable the ADC interrupt
*my_ADCSRA &= 0b11111000; // clear bit 0-2 to 0 to set prescaler selection to slow reading
  // setup the B register
*my_ADCSRB &= 0b11110111; // clear bit 3 to 0 to reset the channel and gain bits
*my_ADCSRB &= 0b11111000; // clear bit 2-0 to 0 to set free running mode
  // setup the MUX Register
*my_ADMUX &= 0b01111111; // clear bit 7 to 0 for AVCC analog reference
*my_ADMUX |= 0b01000000; // set bit 6 to 1 for AVCC analog reference
*my_ADMUX &= 0b11011111; // clear bit 5 to 0 for right adjust result
*my_ADMUX &= 0b11100000; // clear bit 4-0 to 0 to reset the channel and gain bits
}

unsigned int adc_read(unsigned char adc_channel_num){
  // clear the channel selection bits (MUX 4:0)
*my_ADMUX &= 0b11100000;
  // clear the channel selection bits (MUX 5)
*my_ADCSRB &= 0b11110111;
  // set the channel number
  if(adc_channel_num > 7){
    // set the channel selection bits, but remove the most significant bit (bit 3)
adc_channel_num -= 8;
    // set MUX bit 5
*my_ADCSRB |= 0b00001000;
  }
  // set the channel selection bits
*my_ADMUX += adc_channel_num;
  // set bit 6 of ADCSRA to 1 to start a conversion
*my_ADCSRA |= 0x40;
  // wait for the conversion to complete
  while((*my_ADCSRA & 0x40) != 0);
  // return the result in the ADC data register
  return *my_ADC_DATA;
}

//UART FUNCTIONS
void U0init(unsigned long U0baud){
unsigned long FCPU = 16000000;
unsigned int tbaud;
tbaud = (FCPU / 16 / U0baud - 1);
*myUCSR0A = 0x20;
*myUCSR0B = 0x18;
*myUCSR0C = 0x06;
*myUBRR0 = tbaud;
}

unsigned char U0kbhit(){
  return (RDA & *myUCSR0A);
}

unsigned char U0getChar(){
  return *myUDR0;
}

void U0putChar(unsigned char U0pdata){
  while(!(TBE & *myUCSR0A));
*myUDR0 = U0pdata;
}
