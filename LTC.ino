#define ICP1 4 // ICP1, 8 for atmega368, 4 for atmega32u4
#define LTC_OUT 9
#define SIGNAL_LED 7
#define LOCK_LED 6

#define BIT_TIME_THRESHOLD 700
#define BIT_TIME_MIN 250
#define BIT_TIME_MAX 1500

typedef byte LTCFrame[10];

// store 2 frame
volatile LTCFrame frames[2] = {
  {0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF},
  {0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFC, 0xBF}
};

volatile byte currentFrameIndex; // current frame written by ISR
volatile boolean frameAvailable;
volatile unsigned long validFrameCount;

volatile unsigned short validBitCount;

#define NOSYNC 0
#define SYNCED 1
#define GENERATOR 2
volatile char state = 0;

static unsigned short syncPattern = 0xBFFC;// (B00111111 * 256) + B11111101;
volatile unsigned short syncValue;

volatile byte frameBitCount;

volatile byte oneFlag = 0;
volatile unsigned int bitTime;
volatile byte currentBit;
volatile byte lastBit;
ISR(TIMER1_CAPT_vect) {
  TCCR1B ^= _BV(ICES1); // toggle edge capture
  bitTime = ICR1; // store counter value at edge
  TCNT1 = 0; // reset counter

  // remove out of range value
  if ((bitTime < BIT_TIME_MIN) || (bitTime > BIT_TIME_MAX)) {
    // reset everything
    syncValue = 0;
    validBitCount = 0;
    validFrameCount = 0;
    frameAvailable = false;
    state = NOSYNC;
    return;
  }
  
  // increment valid bit counts, without overflow
  validBitCount = validBitCount < 65535 ? validBitCount + 1 : 0; 

  currentBit = bitTime > BIT_TIME_THRESHOLD ? 0 : 1;

  // don't count 1 twice!
  if(currentBit == 1 && lastBit == 1) {
    lastBit = 0;
    return;
  }
  lastBit = currentBit;

  // update frame sync pattern detection
  syncValue = (syncValue >> 1) + (currentBit << 15);

  // update state
  switch(state){
    case NOSYNC:
      // sync pattern detected
      if(syncValue == syncPattern) {
        state = SYNCED;
        frameBitCount = 0;
      }
    break;
   case SYNCED:
    if((frameBitCount > 79 && syncValue != syncPattern)
    || (syncValue == syncPattern && frameBitCount != 79)) {
        // something went wrong!
        syncValue = 0;
        validBitCount = 0;  
        validFrameCount = 0;
        frameAvailable = false;
        state = NOSYNC;
    }
    else if(syncValue == syncPattern) {
      // send the time code
      frameAvailable = true;
      
      // reset bit counter
      frameBitCount = 0;
      
      validFrameCount++;
      currentFrameIndex = 1 - currentFrameIndex;
    }
    else {
      // update ltc data
      byte idx = frameBitCount / 8;
      byte bIdx = frameBitCount & 0x07;
      byte *f = frames[currentFrameIndex];

      f[idx] = (f[idx] & ~(1 << bIdx)) | (currentBit << bIdx);

      /*
      if(currentBit)
        f[idx] |= 1 << bIdx;
       else
        f[idx] &= ~(1 << bIdx);
      */
      

      frameBitCount++;
    }
    break;
 }
}

ISR(TIMER1_OVF_vect) {
  switch(state) {
    case GENERATOR:
    break;
    default:
      // if we overflow, we then lost signal
      syncValue = 0;
      validBitCount = 0;
      validFrameCount = 0;
      frameAvailable = false;
      state = NOSYNC;
      break;
  }
}

struct LTCGenerator {
  volatile byte bitIndex;
  volatile byte bitToggle;
  volatile LTCFrame outFrames[2];
  volatile byte currentFrameIndex; // current frame read by ISR
  volatile bool generateNewFrame;

  int frame;
  int seconds;
  int minutes;
  int hours;
  
  LTCGenerator() {
    reset();
  }

  void reset() {
    bitIndex = 0;
    bitToggle = 0;
    currentFrameIndex = 0;

    // init frame data
    memset(outFrames[0], 0, 10);
    outFrames[0][8] = 0xFC; outFrames[0][9] = 0xBF; // sync pattern
    
    memset(outFrames[1], 0, 10);
    outFrames[1][8] = 0xFC; outFrames[1][9] = 0xBF; // sync pattern

    generateNewFrame = false;

    frame = 0;
    seconds = 0;
    minutes = 0;
    hours = 0;
  }

  // send current frame (should be call inside an ISR)
  void interupt() {
    // toggle for 0 and 1;
    if(!bitToggle) {
      PORTB ^= (1 << 5);

      if(bitIndex == 0)
        PORTD ^= (1 << 7); // debug, use to trigger the scope
    }
    else {
      byte idx = bitIndex / 8;
      byte bitIdx = bitIndex & 0x07;
      byte value = outFrames[currentFrameIndex][idx] & (1 << bitIdx);
      
      // toggle for 1
      if(value)
        PORTB ^= (1 << 5);

        bitIndex++;
        if(bitIndex >= 80) {
          // reset read position
          bitIndex = 0;

          // swtich frame
          currentFrameIndex = 1 - currentFrameIndex;

          generateNewFrame = true;
        }
    }

    bitToggle ^= 1; // toggle
  }

  void update() {
    if(generateNewFrame) {
      generateNewFrame = false;
      frame++;
      if(frame > 30) {
        frame = 0;
        seconds++;
      }

      if(seconds > 60) {
        minutes++;
        seconds = 0;
      }

      if(minutes > 60) {
        hours++;
        minutes = 0;
      }

      // generate frame data
      outFrames[1 - currentFrameIndex][0] = (frame % 10) & 0xf;
      outFrames[1 - currentFrameIndex][1] = (frame / 10) & 0x3;
      outFrames[1 - currentFrameIndex][2] = (seconds % 10) & 0xf;
      outFrames[1 - currentFrameIndex][3] = (seconds / 10) & 0x7;
      outFrames[1 - currentFrameIndex][4] = (minutes % 10) & 0xf;
      outFrames[1 - currentFrameIndex][5] = (minutes / 10) & 0x7;
      outFrames[1 - currentFrameIndex][6] = (hours % 10) & 0xf;
      outFrames[1 - currentFrameIndex][7] = (hours / 10) & 0x3;
    }
  }

} generator;

ISR(TIMER1_COMPA_vect) {
  generator.interupt();
}

void startLTCDecoder() {
  noInterrupts();
  TCCR1A = B00000000; // clear all
  TCCR1B = B11000010; // ICNC1 noise reduction + ICES1 start on rising edge + CS11 divide by 8
  TCCR1C = B00000000; // clear all
  TIMSK1 = B00100001; // ICIE1 (bit 5) enable the icp, and TOIE1 (bit 0) for the overflow

  TCNT1 = 0; // clear timer1

  validBitCount = 0;
  bitTime = 0;
  syncValue = 0;
  currentBit =  0;
  lastBit = 0;
  currentFrameIndex = 0;
  validFrameCount = 0;
  frameAvailable = false;
  state = NOSYNC;
  interrupts();
}

void stopLTCDecoder() {
  noInterrupts();
  TIMSK1 &= ~(1 << ICIE1);
  TIMSK1 &= ~(1 << TOIE1);
  
  TCCR1B &= ~(1<< CS12);
  TCCR1B &= ~(1<< CS11);
  TCCR1B &= ~(1<< CS10);
  interrupts();

  digitalWrite(SIGNAL_LED, LOW); // valid after 1 frame
  digitalWrite(LOCK_LED, LOW);
  
  frameAvailable = false;
  state = NOSYNC;
}


void startLTCGenerator() {
  noInterrupts(); 
  TCCR1A = 0; // clear all
  TCCR1B = (1 << WGM12) | (1 << CS10);
  TIMSK1 = (1 << OCIE1A);

  TCNT1 = 0; 

  // 30 fps, 80 bits * 30
  OCR1A = 3333; // = 16000000 / (30 * 80 * 2)

  state = GENERATOR;
  generator.reset();
  interrupts();
}

void setup() {
  Serial.begin(115200);
  pinMode(ICP1, INPUT);                  // ICP pin (digital pin 8 on arduino) as input
  pinMode(LTC_OUT, OUTPUT);

  pinMode(SIGNAL_LED, OUTPUT);
  pinMode(LOCK_LED, OUTPUT);

  digitalWrite(SIGNAL_LED, LOW);
  digitalWrite(LOCK_LED, LOW);

  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB port only
  }
  Serial.println("Ready!");

  //startLTCDecoder();
  startLTCGenerator();
}

int previousOutputFrameIndex = 0;

void loop() {
  // status led
  if(state != GENERATOR) {
    digitalWrite(SIGNAL_LED, validBitCount > 80 ? HIGH : LOW); // valid after 1 frame
    digitalWrite(LOCK_LED, state == SYNCED? HIGH : LOW);

    if(frameAvailable) {
      byte *fptr = frames[1 - currentFrameIndex];
      Serial.print("Frame: ");
      Serial.print(validFrameCount - 1);

      Serial.print(" - ");

      byte h = (fptr[7] & 0x03) * 10 + (fptr[6] & 0x0F);
      byte m = (fptr[5] & 0x07) * 10 + (fptr[4] & 0x0F);
      byte s = (fptr[3] & 0x07) * 10 + (fptr[2] & 0x0F);
      byte f = (fptr[1] & 0x03) * 10 + (fptr[0] & 0x0F);

      // hours
      Serial.print(h);
      Serial.print(":");
      Serial.print(m);
      Serial.print(":"); // seconds
      Serial.print(s);
      Serial.print(":"); // frame
      Serial.println(f);
      frameAvailable = false;
    }
  }
  else {
    generator.update();
  }
}
