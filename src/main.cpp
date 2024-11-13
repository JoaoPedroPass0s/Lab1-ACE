#include <Arduino.h>
#include <Bounce2.h>

#define MAXIMUM_NUM_NEOPIXELS 5
#include <NeoPixelConnect.h>

#define LED_PIN 6
#define Sgo_pin 2
#define Smore_pin 3
#define Sesc_pin 4

Bounce debouncerSgo = Bounce();
Bounce debouncerSmore = Bounce();
Bounce debouncerSesc = Bounce();

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
const int r[] = {0, 0, 10, 10}; // red values
const int g[] = {0, 10, 10, 10}; // green values
const int b[] = {10, 0, 0, 10}; // blue values

// fsm1 states
enum {
  sm1_off = 0,
  sm1_timer,
  sm1_blink,
  sm1_config
};

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
    fsm.tup = millis();
    fsm.start = true;
    fsm.pause = false;
  }
}

void printOnSerial(const char* message, int value){
  Serial.print(message);
  if(value != -1){
    Serial.print(value);
  }  
  Serial.print("\n");
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
    case 'A':
      config.timer_time = 3;
      break;
    }
    if(Sgo){
      config.timer_time++;
    }
    if(config.timer_time > 3){
      config.timer_time = 0;
    }
    blink_leds(10,0,0,0,0,250);
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
    blink_leds(0,10,0,0,0,250);
  }else if(config_mode == 2){
    switch (serialInput)
    {
    case 'B':
      config.counting_color = 0;
      break;
    case 'G':
      config.counting_color = 1;
      break;
    case 'Y':
      config.counting_color = 2;
      break;
    case 'W':
      config.counting_color = 3;
      break;  
    }
    if(Sgo){
      config.counting_color++;
    }
    if(config.counting_color > 3){
      config.counting_color = 0;
    }
    blink_leds(0,0,10,0,0,250);
  }

  if(Sgo || Smore || serialInput != 0){
    printOnSerial("Config Mode:",config_mode);
    printOnSerial("Timer Time: ",config.timer_time);
    printOnSerial("Counting Effect: ",config.counting_effect);
    printOnSerial("Counting Color: ",config.counting_color);
    printOnSerial("                ",-1);
  }
}

void pause_timer(const uint32_t currentTime){
  fsm1.pause = !fsm1.pause;
  if(fsm1.pause){
    start_pause_time = currentTime;
    last_pause_tup = fsm1.tup;
    fsm1.tup = 0;
    printOnSerial("Before Pause: ",fsm1.tis);
  }else{
    total_pause_time += currentTime - start_pause_time;
    fsm1.tis = currentTime - fsm1.tes - total_pause_time;
    fsm1.tup = last_pause_tup;
    for(int i = 0; i <= led; i++){ // Turn on all leds left on the timer
      strip.neoPixelSetValue(i, r[config.counting_color], g[config.counting_color], b[config.counting_color],false);
    }
    strip.neoPixelShow();
    printOnSerial("After Pause: ",fsm1.tis);
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

  debouncerSgo.attach(Sgo_pin);
  debouncerSgo.interval(50); // 50 ms debounce delay
  debouncerSmore.attach(Smore_pin);
  debouncerSmore.interval(50);
  debouncerSesc.attach(Sesc_pin);
  debouncerSesc.interval(50);
}

void loop() 
{

  if (Serial.available() > 0) {
    // Read a single character
    serialInput = Serial.read();
  }  

  debouncerSgo.update();
  debouncerSmore.update();
  debouncerSesc.update();

  Sgo = debouncerSgo.fell();
  Smore = debouncerSmore.fell();
  Sesc = debouncerSesc.fell();

  // Check if the button is pressed for 3 seconds
  if (!debouncerSmore.read()) {
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

  // State transitions
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
    config = temp_config; // Restore the previous config
    // Print the config
    printOnSerial("Config Mode:",config_mode);
    printOnSerial("Timer Time: ",config.timer_time);
    printOnSerial("Counting Effect: ",config.counting_effect);
    printOnSerial("Counting Color: ",config.counting_color);
  }else if(fsm1.state == sm1_timer && led < 0) { // Timer is done
    printOnSerial("Timer Done: ",fsm1.tis);
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
    printOnSerial("More Time",-1);
    if(led < MAXIMUM_NUM_NEOPIXELS - 1){
      led++; 
      strip.neoPixelSetValue(led - 1, r[config.counting_color], g[config.counting_color], b[config.counting_color],false);
      strip.neoPixelSetValue(led, r[config.counting_color], g[config.counting_color], b[config.counting_color],true);
    }
  }else if(fsm1.state == sm1_blink && fsm1.tis >= 10000){ // Blink is done
    printOnSerial("Blink Done: ",fsm1.tis);
    fsm1.new_state = sm1_off;
  }

  set_state(fsm1, fsm1.new_state); // Change state if needed

  // State actions
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
      fsm1.tup = fsm1.tis;
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
        printOnSerial("Led: ",led);
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
    blink_leds(10,0,0,0,MAXIMUM_NUM_NEOPIXELS,250);
  }
  
  serialInput = 0; // Reset the input 
}

