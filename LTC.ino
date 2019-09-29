#define ICP1 8 // ICP1 for atmega368, 4 for atmega32u4
#define SIGNAL_LED 7
#define LOCK_LED 6

#define BIT_TIME_THRESHOLD 700
#define BIT_TIME_MIN 250
#define BIT_TIME_MAX 1500

typedef byte LTCFrame[10];

// store 2 frame
volatile LTCFrame frames[2] = {
  {0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFF, 0xFF},
  {0x40, 0x20, 0x20, 0x30, 0x40, 0x10, 0x20, 0x10, 0xFF, 0xFF}
};

volatile byte currentFrameIndex; // current frame written by ISR
volatile boolean frameAvailable;
volatile unsigned long validFrameCount;

volatile unsigned short validBitCount;

#define NOSYNC 0
#define SYNCED 1
volatile char state = 0;

static unsigned short syncPattern = (B00111111 * 256) + B11111101;
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
  validBitCount += validBitCount < 65535 ? validBitCount + 1 : 0; 

  currentBit = bitTime > BIT_TIME_THRESHOLD ? 0 : 1;

  // don't count 1 twice!
  if(currentBit == 1 && lastBit == 1) {
    lastBit = 0;
    return;
  }
  lastBit = currentBit;

  // update frame sync pattern detection
  syncValue = (syncValue << 1) + currentBit;

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

      if(currentBit)
        f[idx] |= 1 << bIdx;
       else
        f[idx] &= ~(1 << bIdx);

      frameBitCount++;
    }
    break;
 }
}

ISR(TIMER1_OVF_vect) {
  // if we overflow, we then lost signal
  syncValue = 0;
  validBitCount = 0;
  validFrameCount = 0;
  frameAvailable = false;
  state = NOSYNC;
}

void setup() {
  Serial.begin(115200);
  pinMode(ICP1, INPUT);                  // ICP pin (digital pin 8 on arduino) as input

  pinMode(SIGNAL_LED, OUTPUT);
  pinMode(LOCK_LED, OUTPUT);

  digitalWrite(SIGNAL_LED, LOW);
  digitalWrite(LOCK_LED, LOW);

  Serial.println("Ready!");

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
  interrupts();
}

void loop() {
  // status led
  digitalWrite(SIGNAL_LED, validBitCount > 80 ? HIGH : LOW); // valid after 1 frame
  digitalWrite(LOCK_LED, state == SYNCED? HIGH : LOW);

  if(frameAvailable) {
    byte *fptr = frames[1 - currentFrameIndex];
    Serial.print("Frame :");
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
