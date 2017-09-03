// use first channel of 16 channels (started from zero)
#define LEDC_CHANNEL_0     0

// use resolution bit (precission) for LEDC timer
#define LEDC_TIMER_RES_BIT  11

// use 38 kHz as a LEDC base frequency
#define LEDC_BASE_FREQ     38000

// fade LED PIN (replace with LED_BUILTIN constant for built-in LED)
#define LED_PIN            27

#define BUTTON_PIN 0  // Pin 0
#define SW_PIN 19
#define IRM_PIN 34

// +-20% の範囲内かどうかを調べるマクロ
#define WITHIN_RANGE(t, n) ((n)*0.8 <= (t) && (t) <= (n) * 1.2)
// +-20% の以上かどうかを調べるマクロ
#define OVER_RANGE(t, n) ((t) <= (n) * 1.2)


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
  putLed(0);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(IRM_PIN, INPUT_PULLUP);
  //Serial.println("\nPress sw.");
}

void putLed(uint8_t v) {
  //v = (v == 0 ? 0 : 255/3);
  v = (v == 0 ? 0 : 255/2);
  //v = (v == 0 ? 255 : 255/2);
  ledcAnalogWrite(LEDC_CHANNEL_0, v);
}


#define FMT_UNKNOWN 0
#define FMT_NEC 1
#define FMT_KADENKYO 2
#define FMT_SONY 3


uint8_t state = 0; // 0:idle, 1:send IR, 2:read IR
int counter = 0;
uint8_t readByte;
uint8_t readPos = 0;
uint8_t remoconData[64];
uint8_t remoconDataLen = 0;
uint8_t format = 0; // 0:unknown 1:nec 2:kadenkyo 3:sony
unsigned long sumTime;
unsigned long sumT = 0;

void loop() {

  if (state == 0) {
    if (digitalRead(BUTTON_PIN) == LOW) {
      state = 1;
    }
    if (digitalRead(IRM_PIN) == LOW) {
      state = 2;
    }
  } else if (state == 1) {
    if (sendIR()) {
      state = 0;
    }
  } else if (state == 2) {
    if (readIR()) {
      state = 0;
    }
  }
}



bool sendIR() {
  // Sharp のテレビの電源ボタン
  #define T 630 // [us]
  //#define T 562 // [us]
  #define TRAILER_T 500000 // [us]
  static uint8_t leader[] = {16,8};
  //uint8_t code[] = {0x55,0x5A,0xF1,0x48,0x68,0x8B};
  static uint8_t code[] = {0xe7, 0x30, 0xe1, 0x1e};
  static uint8_t hilo;
  static uint8_t sendState = 0; // 0:idle 1:send LEADER 2:send DATA 3:trailer hi 4:trailer lo
  static unsigned long prevTime = 0;
  static int codePos = 0;
  static int leaderPos = 0;

  unsigned long time, diff, t;
  uint8_t byte, bit;
  time = micros();
  diff = time - prevTime;

  if (sendState == 0) {
    sendState = 1;
    leaderPos = 0;
    Serial.println("sending IR.");
    prevTime = micros();
  } else if (sendState == 1) {
    hilo = ((leaderPos & 0x01) == 0);
    putLed(hilo);
    if (leader[leaderPos]*T <= diff) {
      leaderPos++;
      prevTime = time;
      if (leaderPos >= sizeof(leader)/sizeof(leader[0])) {
        sendState = 2;
        codePos = 0;
        hilo = 1;
      }
    }
  } else if (sendState == 2) {
    putLed(hilo);
    byte = code[codePos >> 3];
    bit = byte & (1 << (7 - (codePos & 0x7)));
    if (hilo == 0 && bit != 0)
      t = T * 3;
    else
      t = T;
    if (t <= diff) {
      prevTime = time;
      if (hilo == 0)
        codePos++;
      hilo = !hilo;
      if ((codePos >> 3) >= (sizeof(code)/sizeof(code[0]))) {
        sendState = 3;
      }
    }
  } else if (sendState == 3) {
    putLed(1);
    if (T <= diff) {
      sendState = 4;
      prevTime = time;
    }
  } else if (sendState == 4) {
    putLed(0);
    if (TRAILER_T <= diff) {
      sendState = 0;
      Serial.println("done.");
      return true;
    }
  }
  return false;
}


bool readIR() {
  static uint8_t readState = 0; // 0:initial 1:reading leader 2:reading data 9:wait 100msec
  static uint8_t hilo;
  static unsigned long prevTime;
  static unsigned long leader[2];
  unsigned long t,diff;

  if (readState == 0) {
    prevTime = micros();
    readState = 1;
    hilo = LOW;
    readPos = 0;
    readByte = 0;
    remoconDataLen = 0;
    sumTime = 0;
    sumT = 0;
  } else if (readState == 1) {
    if (digitalRead(IRM_PIN) != hilo) {
      t = micros();
      diff = t - prevTime;
      if (hilo == LOW) {
        leader[0] = diff;
      } else {
        leader[1] = diff;
        format = parseLeader(leader);
        readState = (format == FMT_UNKNOWN ? 9 : 2);
      }
      hilo = !hilo;
      prevTime = t;
    }
  } else if (readState == 2) {
    if (digitalRead(IRM_PIN) != hilo) {
      t = micros();
      diff = t - prevTime;
      uint8_t ret = parseData(hilo, diff, format);
      if (ret == -1) {
        readState = 9;
      } else if (ret == 0) {
        readState = 3;
      }
    }
  } else if (readState == 9) {
    t = micros();
    if (t - prevTime > 200000) {
      readState = 0;
      return true;
    }
  }
  return false;
}

uint8_t parseLeader(unsigned long *leader) {
  Serial.print("leader : ");

  // NEC T=562us [16T,8T]
  // Kadenkyo T=425us (350-500us) [8T,4T]
  // Sony T=600us [4T,1T]
  #define TNEC 562
  #define TKADENKYO 425
  #define TSONY 600
  if (WITHIN_RANGE(leader[0], 16 * TNEC) && WITHIN_RANGE(leader[1], 8 * TNEC)) {
    Serial.print("NEC => ");
    return FMT_NEC;
  } else if (WITHIN_RANGE(leader[0], 8 * TKADENKYO) &&
             WITHIN_RANGE(leader[1], 4 * TKADENKYO)) {
    Serial.print("KADENKYO => ");
    return FMT_KADENKYO;
  } else if (WITHIN_RANGE(leader[0], 4 * TSONY) &&
             WITHIN_RANGE(leader[1], 1 * TSONY)) {
    Serial.print("SONY => ");
    return FMT_SONY;
  }
  Serial.print(leader[0]);
  Serial.print(",");
  Serial.print(leader[1]);
  Serial.println();
  return FMT_UNKNOWN;
}

/**
 * @return -1:error 0:keep reading 1:end
 */
uint8_t parseData(uint8_t hilo, unsigned long time, uint8_t format) {
  uint8_t bit = 0xff;
  if (format == FMT_SONY) {
    // TODO:
    if (hilo == LOW) {
      if (WITHIN_RANGE(time, 1 * TSONY)) {
      } else if (WITHIN_RANGE(time, 2 * TSONY)) {
        readByte |= (1 << (7-readPos));
      } else {
        return -1;
      }
      readPos++;
    } else {
      if (!WITHIN_RANGE(time, 1 * TSONY)) {
        return -1;
      }
    }
  } else if (format == FMT_NEC) {
    if (hilo == LOW) {
      if (!WITHIN_RANGE(time, 1 * TNEC)) {
        return -1;
      }
      return 0;
    } else {
      if (WITHIN_RANGE(time, 1 * TNEC)) {
      } else if (WITHIN_RANGE(time, 3 * TNEC)) {
        readByte |= (1 << (7-readPos));
      } else if (OVER_RANGE(time, 3 * TNEC)) {
        if (readPos == 0)
          return 1;
        else
          return -1;
      }
    }
  } else if (format == FMT_KADENKYO) {
    if (hilo == LOW) {
      if (!WITHIN_RANGE(time, 1 * TKADENKYO)) {
        return -1;
      }
      return 0;
    } else {
      if (WITHIN_RANGE(time, 1 * TKADENKYO)) {
      } else if (WITHIN_RANGE(time, 3 * TKADENKYO)) {
        readByte |= (1 << (7-readPos));
      } else if (OVER_RANGE(time, 3 * TKADENKYO)) {
        if (readPos == 0)
          return 1;
        else
          return -1;
      }
    }
  }
  readPos++;
  if (readPos == 8) {
    remoconData[remoconDataLen++] = readByte;
    readByte = 0;
    readPos = 0;
  }
  return 0;
}
