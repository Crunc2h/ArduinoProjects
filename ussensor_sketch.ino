#include <LiquidCrystal.h>

//---SERIAL VARIABLES---//

const unsigned long SERIAL_DEF_BAUD = 115200;


//---US SENSOR VARIABLES ---//

const int US_TRIGGER_PIN = 4;
const int US_ECHO_PIN = 3;

const unsigned long US_SIGNAL_SETUP_DELAY = 10;
const unsigned long US_TRIGGER_DELAY = 100000;
const double US_MICROS_TO_CM_RATIO = 58.0;
const double US_REMAINDER_MAX_DISTANCE_CM = 400;
const double US_MAX_DISTANCE_CM = 150;
const double US_MID_DISTANCE_CM = 65;
const double US_CLOSE_DISTANCE_CM = 15;
const double US_MIN_DISTANCE_CM = 2;
const double US_IGNORE_OBJ_THRESHOLD_DISTANCE_CM = 145;
const float US_OLD_DISTANCE_UPDATE_FACTOR = 0.1;
const float US_NEW_DISTANCE_UPDATE_FACTOR = 0.9;


unsigned long us_t_prev_sensor_trigger = micros();
bool us_f_obj_detected = false;
double us_current_obj_distance = 0;

//***US SENSOR INTERRUPT***//
volatile unsigned long us_t_pulse_in_start = micros();
volatile unsigned long us_t_pulse_in_end = micros();
volatile bool us_f_dist_update_available = false;


//---LCD VARIABLES---//

const int LCD_RS_PIN = A5;
const int LCD_E_PIN = A4;
const int LCD_DB4_PIN = 6;
const int LCD_DB5_PIN = 7;
const int LCD_DB6_PIN = 8;
const int LCD_DB7_PIN = 9;
const unsigned long LCD_REFRESH_DELAY = 500000;

LiquidCrystal lcd(LCD_RS_PIN, LCD_E_PIN, LCD_DB4_PIN,
                  LCD_DB5_PIN, LCD_DB6_PIN, LCD_DB7_PIN);
unsigned long lcd_t_prev_refresh = micros();


//---LED VARIABLES---//

const int NUM_OF_LEDS = 4;
const int LED_ALL_PINS[NUM_OF_LEDS] = {10, 11, 12, 13};
const int LED_FAR_DISTANCE_PIN = 10;
const int LED_MID_DISTANCE_PIN = 11;
const int LED_CLOSE_DISTANCE_PIN = 12;
const int LED_OBJ_DET_PIN = 13;

const unsigned long LED_DEF_BLINK_DELAY = 1500000;
const unsigned long LED_DEF_BLINK_TIME = 250000;
const float LED_MAX_BLINK_FREQ_MODIFIER = 6;

unsigned long led_t_prev_blink = micros();
unsigned long led_current_blink_delay = 1000000;
float led_current_blink_freq = 1;
int led_current_target_distance_pin = 10;


//---PIEZO VARIABLES---//

const unsigned long PIEZO_AUDIO_DURATION = 100;
const int PIEZO_FAR_TONE_HZ = 3500;
const int PIEZO_MID_TONE_HZ = 3750;
const int PIEZO_CLOSE_TONE_HZ = 4000;

int piezo_current_tone_hz = 3500;


void setup()
{
  Serial.begin(SERIAL_DEF_BAUD);

  pinMode(US_TRIGGER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(US_ECHO_PIN),
                                        usEchoPulseInterrupt,
                                        CHANGE);
  
  for (int i = 0; i < NUM_OF_LEDS; i++) {
  	pinMode(LED_ALL_PINS[i], OUTPUT);
  }
  
  lcd.begin(16,2);
  printLcdStartMsg(lcd);
}

void loop()
{
  unsigned long t_now = micros();
  
  if (t_now - us_t_prev_sensor_trigger
      > US_TRIGGER_DELAY) {
  	
      us_t_prev_sensor_trigger = t_now;
      triggerUsSensor();
  }
  
  if (us_f_dist_update_available) {
    
    us_f_dist_update_available = false;
    
    double new_obj_distance = getDistanceCm(
      us_t_pulse_in_start,
      us_t_pulse_in_end);

    if ((new_obj_distance >= US_MIN_DISTANCE_CM && new_obj_distance <= US_MAX_DISTANCE_CM) 
    || (new_obj_distance >= US_MIN_DISTANCE_CM && new_obj_distance <= US_REMAINDER_MAX_DISTANCE_CM)) {
  	  us_current_obj_distance = US_OLD_DISTANCE_UPDATE_FACTOR * us_current_obj_distance + US_NEW_DISTANCE_UPDATE_FACTOR * new_obj_distance;
      Serial.println(us_current_obj_distance);
    }

    if (us_current_obj_distance < US_IGNORE_OBJ_THRESHOLD_DISTANCE_CM) {
      
        us_f_obj_detected = true;
      
        led_current_blink_freq = convertDistanceToFrequency(us_current_obj_distance);
    
        led_current_blink_delay = getBlinkDelayByFrequency(led_current_blink_freq);
    
        led_current_target_distance_pin = determineLedTargetDistancePin(us_current_obj_distance);
      
        closeNonTargetDistanceLeds(led_current_target_distance_pin);
      }
      else {
        us_f_obj_detected = false;
      }
  }
  
  if(us_f_obj_detected) {
  	
      if(t_now - led_t_prev_blink > led_current_blink_delay) {
  	    
        led_t_prev_blink = t_now;
        digitalWrite(led_current_target_distance_pin, HIGH);
        tone(led_current_target_distance_pin, 
             piezo_current_tone_hz, 
             PIEZO_AUDIO_DURATION);
  	}
  
  	if(t_now - led_t_prev_blink > LED_DEF_BLINK_TIME) {
  	  digitalWrite(led_current_target_distance_pin, LOW);
  	}
    
    if(digitalRead(LED_OBJ_DET_PIN) == LOW) {
      digitalWrite(LED_OBJ_DET_PIN, HIGH);
    }
  }
  else {
  	closeAllLeds();
  }
  
  if (t_now - lcd_t_prev_refresh > LCD_REFRESH_DELAY) {
  	lcd_t_prev_refresh = t_now;
    refreshLcdScreen(lcd, 
                       us_f_obj_detected, 
                       us_current_obj_distance);
  }
}


void printLcdStartMsg(LiquidCrystal lcd_obj) {
  delay(2000);
  lcd_obj.setCursor(2, 0);
  lcd_obj.print("USSensor V0.1");
  delay(2000);
  lcd_obj.setCursor(5, 1);
  lcd_obj.print("Welcome!");
  delay(2000);
}

void refreshLcdScreen(LiquidCrystal lcd_obj, 
                      bool obj_detected, 
                      double current_obj_distance) {
  lcd_obj.clear();
  if (obj_detected) {
    lcd_obj.setCursor(0, 0);
    lcd_obj.print("Object Detected!");
    lcd_obj.setCursor(0, 1);
    lcd_obj.print("Dis. -> " + String(current_obj_distance) + "cm");
  }
  else {
    lcd_obj.setCursor(4,0);
    lcd_obj.print("No Object");
    lcd_obj.setCursor(4,1);
    lcd_obj.print("Detected!");
  }
}


int determineLedTargetDistancePin(double obj_distance) {
  if (obj_distance > US_MID_DISTANCE_CM && obj_distance < US_MAX_DISTANCE_CM) {
  	piezo_current_tone_hz = PIEZO_FAR_TONE_HZ;
    return LED_FAR_DISTANCE_PIN;
  }
  else if (obj_distance > US_CLOSE_DISTANCE_CM && obj_distance < US_MID_DISTANCE_CM) {
  	piezo_current_tone_hz = PIEZO_MID_TONE_HZ;
    return LED_MID_DISTANCE_PIN;
  }
  else {
  	piezo_current_tone_hz = PIEZO_CLOSE_TONE_HZ;
    return LED_CLOSE_DISTANCE_PIN;
  }
}

void closeAllLeds() {
  for (int i = 0; i < NUM_OF_LEDS; i++) {
  	digitalWrite(LED_ALL_PINS[i], LOW);
  }
}

void closeNonTargetDistanceLeds(int current_target_distance_pin) {
  for (int i = 0; i < NUM_OF_LEDS; i++) {
  	if (LED_ALL_PINS[i] != current_target_distance_pin 
        && LED_ALL_PINS[i] != LED_OBJ_DET_PIN) {
    	digitalWrite(LED_ALL_PINS[i], LOW);
    }
  }
}

float getBlinkDelayByFrequency(float current_freq) {
  return LED_DEF_BLINK_DELAY / current_freq;
}                                         


float convertDistanceToFrequency(double obj_distance) { 
  
  float updated_frequency = LED_MAX_BLINK_FREQ_MODIFIER 
    - (obj_distance / US_MAX_DISTANCE_CM) * LED_MAX_BLINK_FREQ_MODIFIER;

  return updated_frequency;
}



double getDistanceCm(unsigned long t_pulse_in_start,
                     unsigned long t_pulse_in_end) {
  double echo_pulse_duration = t_pulse_in_end - t_pulse_in_start;
  double obj_distance_cm = echo_pulse_duration / US_MICROS_TO_CM_RATIO;
  return obj_distance_cm;
}

void triggerUsSensor() {
  digitalWrite(US_TRIGGER_PIN, LOW);
  delayMicroseconds(US_SIGNAL_SETUP_DELAY);
  digitalWrite(US_TRIGGER_PIN, HIGH);
  delayMicroseconds(US_SIGNAL_SETUP_DELAY);
  digitalWrite(US_TRIGGER_PIN, LOW);
}

void usEchoPulseInterrupt() {
  unsigned long t_now = micros();
  if (digitalRead(US_ECHO_PIN) == HIGH) {
  	us_t_pulse_in_start = t_now;
  }
  else {
  	us_t_pulse_in_end = t_now;
    us_f_dist_update_available = true;
  }
}


  








