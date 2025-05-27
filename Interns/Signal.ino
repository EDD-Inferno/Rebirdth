#include <FastLED.h>
#define LED_PIN     6
#define NUM_LEDS    256     // 16 x 16 = 256 LEDs
#define BRIGHTNESS  5       // Brightness level (0-255)
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB
CRGB leds[NUM_LEDS];
void setup() {
 FastLED.addLeds<LED_TYPE, LED_PIN, COLOR_ORDER>(leds, NUM_LEDS);
 FastLED.setBrightness(BRIGHTNESS);
 // Blink Red for 8 seconds (4 blinks, 1s on + 1s off = 2s per blink)
 for (int i = 0; i < 4; i++) {
   fill_solid(leds, NUM_LEDS, CRGB::Red);
   FastLED.show();
   delay(1000);  // 1 second ON
   fill_solid(leds, NUM_LEDS, CRGB::Black);
   FastLED.show();
   delay(1000);  // 1 second OFF
 }
 // Yellow for 1 second
 fill_solid(leds, NUM_LEDS, CRGB::Yellow);
 FastLED.show();
 delay(1000);
 // Green for 1 second
 fill_solid(leds, NUM_LEDS, CRGB::Green);
 FastLED.show();
 delay(1000);
}
void loop() {
 // Intentionally left empty
}
