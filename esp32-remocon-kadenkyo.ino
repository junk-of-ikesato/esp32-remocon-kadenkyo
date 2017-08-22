// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use resolution bit (precission) for LEDC timer
#define LEDC_TIMER_RES_BIT  11

// use 38 kHz as a LEDC base frequency
#define LEDC_BASE_FREQ     38000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            12


// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 2047 from 2 ^ 11 - 1
  uint32_t duty = (2047 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void setup() {
  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_RES_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  ledcAnalogWrite(LEDC_CHANNEL_0, 256/3);
}

void loop() {
}
