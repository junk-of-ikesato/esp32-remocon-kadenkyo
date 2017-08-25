// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use resolution bit (precission) for LEDC timer
#define LEDC_TIMER_RES_BIT  11

// use 38 kHz as a LEDC base frequency
#define LEDC_BASE_FREQ     38000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            12

#define BUTTON_PIN 0  // Pin 0


// Arduino like analogWrite
// value has to be between 0 and valueMax
void ledcAnalogWrite(uint8_t channel, uint32_t value, uint32_t valueMax = 255) {
  // calculate duty, 2047 from 2 ^ 11 - 1
  uint32_t duty = (2047 / valueMax) * min(value, valueMax);

  // write duty to LEDC
  ledcWrite(channel, duty);
}

void setup() {
  Serial.begin(115200);
  // Setup timer and attach timer to a led pin
  ledcSetup(LEDC_CHANNEL_0, LEDC_BASE_FREQ, LEDC_TIMER_RES_BIT);
  ledcAttachPin(LED_PIN, LEDC_CHANNEL_0);
  ledcAnalogWrite(LEDC_CHANNEL_0, 0);
  Serial.println("\nPress sw.");
}

void putLed(uint8_t v) {
  v = (v == 0 ? 0 : 256/3);
  ledcAnalogWrite(LEDC_CHANNEL_0, v);
}


// Sharp のテレビの電源ボタン
#define T 425 // [us]
#define TRAILER_T 69000 // [us]
uint8_t leader[] = {8,4};
uint8_t code[] = {0x55,0x5A,0xF1,0x48,0x68,0x8B};
unsigned long prevTime = 0;
int state = 0; // 0:idle 1:send LEADER 2:send DATA 3:trailer hi 4:trailer lo
int codePos = 0;
int leaderPos = 0;
uint8_t hilo;

void loop() {
  unsigned long time, diff, t;
  uint8_t byte, bit;
  time = micros();
  diff = time - prevTime;

  if (state == 0) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      state = 1;
      leaderPos = 0;
      Serial.println("sending IR.");
      prevTime = micros();
    }
  } else if (state == 1) {
    hilo = ((leaderPos & 0x01) == 0);
    putLed(hilo);
    if (leader[leaderPos]*T <= diff) {
      leaderPos++;
      prevTime = time;
      if (leaderPos >= sizeof(leader)/sizeof(leader[0])) {
        state = 2;
        codePos = 0;
        hilo = 1;
      }
    }
  } else if (state == 2) {
    putLed(hilo);
    byte = code[codePos >> 3];
    bit = byte & (1 << (7 - (codePos & 0x7)));
    if (hilo == 0 && bit != 0)
      t = T * 3;
    else
      t = T;
    if (t <= diff) {
//printf("%2d %d 0x%02x 0x%02x %s\n", codePos, hilo, byte, bit, (t==T ? "T" : "3T"));
      prevTime = time;
      if (hilo == 0)
        codePos++;
      hilo = !hilo;
      if ((codePos >> 3) >= (sizeof(code)/sizeof(code[0]))) {
        state = 3;
      }
    }
  } else if (state == 3) {
    putLed(1);
    if (T <= diff) {
      state = 4;
      prevTime = time;
    }
  } else if (state == 4) {
    putLed(0);
    if (TRAILER_T <= diff) {
      state = 0;
      Serial.println("done.");
    }
  }
}
