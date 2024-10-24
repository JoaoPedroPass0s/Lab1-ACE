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
  bool start;

} fsm_t;

// Our finite state machines
fsm_t fsm1;
uint8_t Sgo, Smore, Sesc;
int led;
bool blink_on;

// meaningful names for the fsm1 states
enum {
  sm1_off = 0,
  sm1_timer,
  sm1_blink
};

// Set new state
void set_state(fsm_t& fsm, int new_state)
{
  if (fsm.state != new_state) {  // if the state changed, reset tis
    fsm.state = new_state;
    fsm.tes = millis();
    fsm.tis = 0;
    fsm.tup = millis();
    fsm.start = true;
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

}

void loop() 
{
  // Read the button state
  Sgo = !digitalRead(Sgo_pin);
  Smore = !digitalRead(Smore_pin);
  Sesc = !digitalRead(Sesc_pin);

  // Update the time in state (tis) for the finite state machine
  const uint32_t currentTime = millis();
  fsm1.tis = currentTime - fsm1.tes;

  if (fsm1.state == sm1_off && Sgo){
    led = MAXIMUM_NUM_NEOPIXELS;
    fsm1.new_state = sm1_timer;
  } else if(fsm1.state == sm1_timer && led < 0) {
    Serial.print(fsm1.tis);
    fsm1.new_state = sm1_blink;
    blink_on = true;
  }else if((fsm1.state == sm1_timer) && Sgo) {  
    Serial.print("Reset");
    led = MAXIMUM_NUM_NEOPIXELS;
    fsm1.start = true;
  } else if(fsm1.state == sm1_blink && fsm1.tis >= 5000){
    Serial.print(fsm1.tis);
    fsm1.new_state = sm1_off;
  }

  set_state(fsm1, fsm1.new_state);

  if (fsm1.state == sm1_off){
    if(fsm1.start){
      strip.neoPixelFill(0,0,0,true);
      fsm1.start = false;
    }
  } else if (fsm1.state == sm1_timer){
    if(led == MAXIMUM_NUM_NEOPIXELS && fsm1.start){
      strip.neoPixelFill(50,50,50,true);
      fsm1.start = false;
    }
    if(fsm1.tis-fsm1.tup >= 1000){
      if(led>=0){
        Serial.print(led);
        strip.neoPixelSetValue(led, 0, 0, 0,true);
        led--;
      }
      fsm1.tup = fsm1.tis;
    }
  } else if(fsm1.state == sm1_blink){
    if(fsm1.tis-fsm1.tup >= 250){
      if(blink_on){
        strip.neoPixelFill(0,0,0,true);
        blink_on = false;
      }else{
        strip.neoPixelFill(255,0,0,true);
        blink_on = true;
      }
      fsm1.tup = fsm1.tis;
    }
  }
  
}
