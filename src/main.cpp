#include <Arduino.h>

#define MAXIMUM_NUM_NEOPIXELS 5
#include <NeoPixelConnect.h>

#define LED_PIN 6
#define Sgo_pin 2
#define Smore_pin 3
#define Sesc_pin 4

// Create an instance of NeoPixelConnect and initialize it
// to use GPIO pin 22 as the control pin, for a string
// of 8 neopixels. Name the instance p
NeoPixelConnect strip(LED_PIN, MAXIMUM_NUM_NEOPIXELS, pio0, 0);

typedef struct {
  int state, new_state;

  // tes - time entering state
  // tis - time in state
  // tup - time since last update
  unsigned long tes, tis, tup;
  bool start,pause;

} fsm_t;

typedef struct {
  
  int timer_time = 0;
  int counting_effect = 0;
  int counting_color = 0;
} config_t;

// Our finite state machines
fsm_t fsm1;

config_t config;
config_t temp_config;

uint8_t Sgo, Smore, Sesc;

uint32_t interval, last_cycle;

char serialInput = 0;

int led;
int config_mode;
int current_led_brightness;
int last_led_brightness;

bool blink_on, configMode = false;

unsigned long last_blink = 0;
unsigned long total_pause_time = 0;
unsigned long start_pause_time = 0;
unsigned long last_pause_tup = 0;
unsigned long pressStartTime = 0;

const int intervalOptions[] = {1000, 2000, 5000, 10000};
const int r[] = {0, 0, 255, 255}; // red values
const int g[] = {0, 255, 255, 255}; // green values
const int b[] = {255, 0, 0, 255}; // blue values

// meaningful names for the fsm1 states
enum {
  sm1_off = 0,
  sm1_timer,
  sm1_blink,
  sm1_config
};

typedef struct {
  unsigned long lastDebounceTime;
  bool lastButtonState;
} debounce_t;

debounce_t debounceSgo = {0, HIGH};   // Debounce state for Sgo button
debounce_t debounceSmore = {0, HIGH}; // Debounce state for Smore button
debounce_t debounceSesc = {0, HIGH};  // Debounce state for Sesc button

// Modified debounce function that takes a pointer to debounce_t
bool debounce(uint8_t buttonPin, unsigned long debounceDelay, debounce_t *debounceState) {
  bool currentButtonState = digitalRead(buttonPin);

  if (currentButtonState != debounceState->lastButtonState) {
    debounceState->lastDebounceTime = millis();
  }

  if ((millis() - debounceState->lastDebounceTime) > debounceDelay) {
    debounceState->lastButtonState = currentButtonState;
  }

  return debounceState->lastButtonState;
}
// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed, reset tis
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
    fsm.tup = millis();
    fsm.start = true;
    fsm.pause = false;
  }
}

void blink_leds(int r,int g,int b,int ledS, int ledF, int interval){
  if(fsm1.tis-last_blink >= interval){
    for(int i = ledS; i <= ledF; i++){
      strip.neoPixelSetValue(i, blink_on ? r:0, blink_on ? g:0, blink_on ? b:0,false);
    }
    strip.neoPixelShow();
    blink_on = !blink_on;
    last_blink = fsm1.tis;
  }
}

void process_config(){
  if(fsm1.start){
    Serial.print("Config Mode");
    temp_config = config;
    config_mode = 0;
    fsm1.start = false;
    last_blink = fsm1.tup;
  }

  if(Smore){
    config_mode++;
    if(config_mode > 2){
      config_mode = 0;
    }
  }

  if(config_mode == 0){
    switch (serialInput)
    {
    case '1':
      config.timer_time = 0;
      break;
    case '2':
      config.timer_time = 1;
      break;
    case '5':
      config.timer_time = 2;
      break;
    }
    if(Sgo){
      config.timer_time++;
    }
    if(config.timer_time > 3){
      config.timer_time = 0;
    }
    blink_leds(0,0,255,0,MAXIMUM_NUM_NEOPIXELS,250);
  }else if(config_mode == 1){
    switch (serialInput)
    {
    case 'o':
      config.counting_effect = 0;
      break;
    case 'b':
      config.counting_effect = 1;
      break;
    case 'f':
      config.counting_effect = 2;
      break;
    }
    if(Sgo){
      config.counting_effect++;
    }
    if(config.counting_effect > 2){
      config.counting_effect = 0;
    }
    blink_leds(0,255,0,0,MAXIMUM_NUM_NEOPIXELS,250);
  }else if(config_mode == 2){
    switch (serialInput)
    {
    case 'b':
      config.counting_color = 0;
      break;
    case 'g':
      config.counting_color = 1;
      break;
    case 'y':
      config.counting_color = 2;
      break;
    case 'w':
      config.counting_color = 3;
      break;  
    }
    if(Sgo){
      config.counting_color++;
    }
    if(config.counting_color > 3){
      config.counting_color = 0;
    }
    blink_leds(255,0,0,0,MAXIMUM_NUM_NEOPIXELS,250);
  }

  if(Sgo){
    Serial.print("Timer time :"+config.timer_time);
    Serial.print("Counting effect :"+config.counting_effect);
    Serial.print("Counting colour :"+config.counting_color);
  }
}

void pause_timer(const uint32_t currentTime){
  fsm1.pause = !fsm1.pause;
  if(fsm1.pause){
    start_pause_time = currentTime;
    last_pause_tup = fsm1.tup;
    fsm1.tup = 0;
    Serial.print("Before Pause: ");
    Serial.println(fsm1.tis);
    Serial.print("\n");
  }else{
    total_pause_time += currentTime - start_pause_time;
    fsm1.tis = currentTime - fsm1.tes - total_pause_time;
    fsm1.tup = last_pause_tup;
    for(int i = 0; i <= led; i++){ // Turn on all leds left on the timer
      strip.neoPixelSetValue(i, r[config.counting_color], g[config.counting_color], b[config.counting_color],false);
    }
    strip.neoPixelShow();
    Serial.print("After Pause: ");
    Serial.println(fsm1.tis);
    Serial.print("\n");
  }
}

void setup() 
{
  pinMode(LED_BUILTIN, OUTPUT);
  Serial.begin(115200);
  pinMode(Sgo_pin,INPUT_PULLUP);
  pinMode(Smore_pin,INPUT_PULLUP);
  pinMode(Sesc_pin,INPUT_PULLUP);
  set_state(fsm1, sm1_off); 

  interval = 40;
}

void loop() 
{

  if (Serial.available() > 0) {
    // Read a single character
    serialInput = Serial.read();
  }  

  uint32_t now = millis();
  if (now - last_cycle > interval) {
    last_cycle = now;

    // Read the button state
    Sgo = !digitalRead(Sgo_pin);
    Smore = !digitalRead(Smore_pin);
    Sesc = !digitalRead(Sesc_pin);

    // Check if the button is pressed for 3 seconds
    if (Smore) {
        if (pressStartTime == 0) pressStartTime = millis();
        if (millis() - pressStartTime > 3000){
          configMode = !configMode;
          pressStartTime = 0;
        }  
    } else {
        pressStartTime = 0;
    }

    // Update the time in state (tis) for the finite state machine
    const uint32_t currentTime = millis();
    fsm1.tis = currentTime - fsm1.tes - total_pause_time;

    if (fsm1.state == sm1_off && Sgo){ // If needed, change the next state
      led = MAXIMUM_NUM_NEOPIXELS - 1;
      fsm1.new_state = sm1_timer;
    }else if(fsm1.state == sm1_off && (configMode || serialInput == 'c')){ // Enter config mode
      configMode = true;
      fsm1.new_state = sm1_config;
    }else if(fsm1.state == sm1_config && (!configMode || serialInput == 's')){ // Exit config mode and save
      configMode = false;
      fsm1.new_state = sm1_off;
    }else if(fsm1.state == sm1_config && (Sesc || serialInput == 'e')){ // Exit config mode without saving
      configMode = false;
      fsm1.new_state = sm1_off; 
      config = temp_config; 
    }else if(fsm1.state == sm1_timer && led < 0) { // Timer is done
      Serial.print(fsm1.tis);
      fsm1.new_state = sm1_blink; 
      blink_on = true;
      last_blink = fsm1.tis;
      total_pause_time = 0;
    }else if((fsm1.state == sm1_timer) && ( (Sgo) || (serialInput == 'g') ) ){ // Reset timer
      led = MAXIMUM_NUM_NEOPIXELS - 1; 
      fsm1.start = true;
    }else if((fsm1.state == sm1_timer) && (Sesc || (fsm1.pause? (serialInput == 'r') : (serialInput == 'p')))){
      pause_timer(currentTime); // Pause/Unpause timer
    }else if((fsm1.state == sm1_timer) && ( (Smore) || (serialInput == 'm') )){ // Add more time
      Serial.print("More");
      if(led < MAXIMUM_NUM_NEOPIXELS - 1){
        led++; 
        strip.neoPixelSetValue(led, r[config.counting_color], g[config.counting_color], b[config.counting_color],true);
      }
    }else if(fsm1.state == sm1_blink && fsm1.tis >= 2000){ // Blink is done
      Serial.print(fsm1.tis);
      fsm1.new_state = sm1_off;
    }

    set_state(fsm1, fsm1.new_state); // Change state if needed

    if (fsm1.state == sm1_off){ 
      if(fsm1.start){ 
        strip.neoPixelFill(0,0,0,true); 
        fsm1.start = false;
      }
    } else if(fsm1.state == sm1_config){
      process_config(); // Process inputs to change config
    } else if (fsm1.state == sm1_timer && !fsm1.pause){
      if(led == MAXIMUM_NUM_NEOPIXELS - 1 && fsm1.start){
        strip.neoPixelFill(r[config.counting_color], g[config.counting_color], b[config.counting_color],true);
        fsm1.start = false;
        last_blink = fsm1.tis;
        current_led_brightness = 100; // Set the brightness of the first led to 100%
        last_led_brightness = 100;
      }
      if(config.counting_effect == 2){
        current_led_brightness = 100 - (fsm1.tis-fsm1.tup)*100/intervalOptions[config.timer_time];
        int red = r[config.counting_color] * current_led_brightness / 100; 
        int green = g[config.counting_color] * current_led_brightness / 100;
        int blue = b[config.counting_color] * current_led_brightness / 100;
        if(last_led_brightness != current_led_brightness && current_led_brightness > 0){
          strip.neoPixelSetValue(led, red, green, blue,true);
          last_led_brightness = current_led_brightness;
        }
      }
      if(fsm1.tis-fsm1.tup >= intervalOptions[config.timer_time]){
        if(led>=0){
          Serial.print(led);
          strip.neoPixelSetValue(led, 0, 0, 0,true);
          led--;
        }
        fsm1.tup = fsm1.tis;
        last_blink = fsm1.tis;
        current_led_brightness = 100;
        last_led_brightness = 100;
      }
      if(config.counting_effect == 1 && (fsm1.tis-fsm1.tup >= (intervalOptions[config.timer_time]/2))){
        blink_leds(r[config.counting_color], g[config.counting_color], b[config.counting_color],led,led,50);
      }
    }else if (fsm1.state == sm1_timer && fsm1.pause){
      blink_leds(r[config.counting_color], g[config.counting_color], b[config.counting_color],0,led,250);
    } else if (fsm1.state == sm1_blink) {
      blink_leds(255,0,0,0,MAXIMUM_NUM_NEOPIXELS,250);
    }
    
    serialInput = 0; // Reset the input
  }  
}

