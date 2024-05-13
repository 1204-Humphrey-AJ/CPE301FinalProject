//Authors: Alex Humpherey, Max Dunlay, Ethan Hanson
//Date: 5/10/2024
//Purpose: Programming Swamp Cooler
#include <LiquidCrystal.h>
#include <DHT.h>
#include <RTClib.h>
#include <Stepper.h>
// BLed = 33 Gled = 32 YLED = 31 RLED = 30
//Fan = 7
// Configuration
const int stepper_steps = 2048;
const int steps_to_take = 180;
const int Rev_steps_to_take = -180;
const int rs = 12, en = 11, d4 = 4, d5 = 5, d6 = 3, d7 = 2; // LCD Pins
int onOffState=0;

volatile unsigned char* pin_b = (unsigned char*) 0x23; //assigns registers with Pin B- used for LEDs
volatile unsigned char* ddr_b  = (unsigned char*) 0x24; 
volatile unsigned char* port_b = (unsigned char*) 0x25;

const float temp_level_min = 26;
const int water_level_min = 250;

int red = 0x80;
int yellow = 0x40;
int green = 0x20;
int blue = 0x10;
// End Configuration

// Peripheral Initialization
Stepper small_stepper(stepper_steps, 22, 23, 24, 25);
DHT dht(8, DHT11);
RTC_DS1307 rtc;
LiquidCrystal lcd(rs, en, d4, d5, d6, d7);
// End Peripheral Initialization

// We form a state machine using an enum of possible states.
enum State {
  DISABLED,
  IDLE,
  RUNNING,
  ERROR,
};

// System State
State current_state = IDLE;

float temp_level = 0; //variables used to keep track of inputs
int water_level = 0;
float humid_level = 0;

unsigned long last_lcd_update = 0;
// End System State

// Misc Constants
volatile unsigned char* led = &PORTC;

const unsigned char RDA = 0x80;
const unsigned char TBE = 0x20;
// End Misc Constants

// Transition states according to state diagram.
void change_state(State new_state) {
  if (current_state == new_state) {
    // Nothing has changed, early return.
    return;
  }
  
  if (new_state == RUNNING) {
    // We have transitioned to the running state.
    fan_on();
  }
  
  if (current_state == RUNNING) {
    // We have transitioned away from the running state.
    fan_off();
  }
  
  current_state = new_state;
  update_led();
  update_display();

  String state_name;//displays to serial output
  switch (new_state) {
    case DISABLED:
      state_name = "DISABLED";
      break;
    case IDLE:
      state_name = "IDLE";
      break;
    case RUNNING:
      state_name = "RUNNING";
      break;
    case ERROR:
      state_name = "ERROR";
      break;
  }

  log("State changed to " + state_name);
}

// Update the display using the state variables.
void update_display() {
  if (last_lcd_update >= (millis() - 500)) return; // Only update screen if 500 ms have passed.
  
  if (current_state == ERROR) {
    lcd.clear(); //clear LCD screen
    lcd.setCursor(0, 0);//set cursor @ origin
    lcd.print("Water level low");
    lcd.setCursor(0, 1);
    lcd.print("Check basin!!!!!"); // display error message
    
  } else if (current_state == DISABLED) {
    lcd.clear(); //clear LCD screen
    lcd.setCursor(0,0);//set cursor @ origin
    lcd.print("DISABLED"); //display disabled message
  } else {
    
    lcd.clear(); //clear LCD screen
    lcd.setCursor(0,0);//set cursor @ origin
    lcd.print("Temp: ");
    lcd.print(temp_level);
    lcd.print("C");
    
    lcd.setCursor(0,1);//move the cursor down a row
    lcd.print("Humid: ");
    lcd.print(humid_level);
    lcd.print("%");
  }

  last_lcd_update = millis();
}

// Update the LEDs using the state variable.
void update_led() {
  if (current_state == DISABLED) {//Turns LEDs different colors based on the state of operation
    *led = yellow;
  } else if (current_state == IDLE) {
    *led = green;
  } else if (current_state == RUNNING) {
    *led = blue;
  } else {
    *led = red;
  }
}

void start() {
  if (current_state != DISABLED) {
    // We're not in a disabled state, start does nothing.
    return;
  }
  
  change_state(IDLE);
}

void stop() {
  if (current_state == DISABLED) {
    // We're already in the disabled state, stop does nothing.
    return;
  }
  
  change_state(DISABLED);
}

void reset() {
  if (current_state != ERROR) {
    // We're not in an error state, reset does nothing.
    return;
  }

  if (water_level > water_level_min) {
    // If the water level is above the minnimum level, we can reset.
    change_state(IDLE);
  }
}

void acquire_data() {//acquires data from each sensor and sends to corresponding variables
  float new_temp = dht.readTemperature();
  float new_humid = dht.readHumidity();
  if (!isnan(new_temp)) temp_level = new_temp;
  if (!isnan(new_humid)) humid_level = new_humid;
}

void adc_init() {  
  // setup the A register // Introduction
  // set bit 7 to 1 to enable the ADC
  ADCSRA |= 0b10000000; // sets bit 7 which is ADEN
  // clear bit 5 and bit 3 and bits 2:0 to 0 to disable the ADC trigger mode, the ADC interrupt, and to set prescaler mode to slow reading
  ADCSRA &= 0b11010000;
  
  
  // setup the B register // Introduction
  // clear bit 3 to 0 and bit 2:0 to 0 to reset the channel and gain bits and to set to free running mode
  ADCSRB &= 0b11110000;
  
  // setup the MUX Register // Introduction
  
  // clear bit 7 to 0 for AVCC analog reference
  ADMUX &= 0b01111111;
  // set bit   6 to 1 for AVCC analog reference'
  ADMUX |= 0b01000000;
  // clear bit 5 to 0 for right adjust result
  ADMUX &= 0b11011111;
  ADMUX &= 0b11011111;
  // clear bit 5 to 0 for right adjust result

  // clear bit 4-0 to 0 to reset the channel and gain bits
  ADMUX &= 0b11100000;
}

unsigned int adc_read(unsigned char adc_channel_num) {
  ADMUX &= 0b11100000;
  // clear the channel selection bit

  // clear the channel selection bits bit 3 in ADCSRB
  ADCSRB &= 0b11110111; 
  
  // set the channel number
  if (adc_channel_num > 7) {
    ADCSRB |= 0b00001000;
    adc_channel_num -= 8;
  }

  // set the channel selection bits
  ADMUX += adc_channel_num;
  
  // set bit 6 of ADCSRA to 1 to start a conversion
  ADCSRA |= 0b01000000;
  
  // wait for the conversion to complete
  while ((ADCSRA & 0x40) != 0);
  
  // return the result in the ADC data register
  return ADC;
}

void button_states() {
  if(*pin_b & 0x08){//if pin 50 is high
    onOffState++;
    delay(100);
  }
  if (onOffState = 1){
    start();//runs start function
    onOffState++;
  }
  else if (onOffState = 2){
  } //prevents start from running indefinitely 
  else if(onOffState = 3){
    stop();
    onOffState = 0;
  }
  else{}
  if(*pin_b & 0x04){//if pin 51 is high
    reset(); //run reset function
    delay(100);
  }
  else{}
  if(*pin_b & 0x02){//if pin 52 is high 
      // make fan angle move right
      log("Vent angle increased.");
      small_stepper.step(steps_to_take); 
    }
   if(*pin_b & 0x01){
      // make vent angle move left
      log("Vent angle decreased.");
      small_stepper.step(Rev_steps_to_take);
   }
}

unsigned char read_char() {
  return UDR0;
}

void write_char(unsigned char U0pdata) {
  while ((UCSR0A & TBE)==0){}; // wait for TBE = true
  UDR0 = U0pdata;
}

void uart_init(unsigned long U0baud) {
 unsigned long FCPU = 16000000;
 unsigned int tbaud;
 tbaud = (FCPU / 16 / U0baud - 1);
 // Same as (FCPU / (16 * U0baud)) - 1;
 UCSR0A = 0x20; // Set data register empty
 UCSR0B = 0x18; // enable transmit and recieve
 UCSR0C = 0x06; // set 8 bit
 UBRR0  = tbaud; // Set the baud rate of the output.
}

unsigned char uart_kb() {
  return UCSR0A & RDA;
}

// Output message to the serial port.
void write_string(const String& message) {
  for (int i = 0; i < message.length(); i++) {
    write_char(message[i]);
  }
}

// Output a message alongside a timestamp to the serial port.
void log(const String& message) {
  DateTime now = rtc.now();

  write_string(String(now.timestamp()) + ": " + message + '\n');
}

void fan_setup() {//assigns the pin at which the fan is connected 
  DDRH |= 0x40;
}

void fan_on() {//Turn the fan on
  PORTH |= 0x40;
}

void fan_off() {//turn the fan off 
  PORTH &= ~(0x40);
}
void button_setup() {
  *ddr_b &= 0xF0;//sets what pins correspond to which buttons
  *port_b |= 0x0F;
}
void setup() {
  // Initialize hardware
  lcd.begin(16, 2);
  rtc.begin();
  dht.begin();
  uart_init(9600); // Init uart at 9600 baud
  adc_init();
  fan_setup();
  button_setup();
  DDRC = 0xF0;
  
  small_stepper.setSpeed(10);

  // Initialize outputs.
  log("Started");
  update_led();
  update_display();
}

void loop() {
  // Acquire our data before we do anything.
  acquire_data();
  button_states();
  // Try to update the display if needed
  update_display();
  
  water_level = adc_read(0); // argument = port number (we are using ADC 0 for water level monitoring)
  if (current_state == DISABLED) {
    // We are in the disabled state, Do nothing
    return;
  }
  
  if (current_state == RUNNING || current_state == IDLE) {
    if (water_level < water_level_min) {
      // Water level is below a certain level, switch to error state.
      change_state(ERROR);
      return;
    }
  }

  if (current_state == RUNNING && temp_level <= temp__level_min) {
    // We're running and it's too cold, stop running.
    change_state(IDLE);
    return;
  }

  if (current_state == IDLE && temp_level > temp_level_min) {
    // We're not running and it's too hot, start running.
    change_state(RUNNING);
    return;
  }
}