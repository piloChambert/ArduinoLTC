#define ICP1 8 // ICP1 for atmega368, 4 for atmega32u4
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

volatile byte generatorFramePosition = 0;
volatile byte generatorFrameToggle = 0;
LTCFrame testFrame = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFC, 0xBF};
ISR(TIMER1_COMPA_vect) {
  // toggle for 0 and 1;
  if(!generatorFrameToggle) {
    PORTB ^= (1 << 1);

    if(generatorFramePosition == 0)
      PORTD ^= (1 << 7); // debug, use to trigger the scope
  }
    
  else {
    byte idx = generatorFramePosition / 8;
    byte bitIdx = generatorFramePosition & 0x07;
    byte value = testFrame[idx] & (1 << bitIdx);
    
    // toggle for 1
    if(value)
      PORTB ^= (1 << 1);

      generatorFramePosition++;
      if(generatorFramePosition >= 80)
        generatorFramePosition = 0;
  }

  generatorFrameToggle ^= 1; // toggle
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
  OCR1A = 3333; // 30 fps, 80 bits * 30

  state = GENERATOR;

  generatorFramePosition = 0;
  interrupts();
}

void setup() {
  Serial.begin(115200);
  pinMode(ICP1, INPUT);                  // ICP pin (digital pin 8 on arduino) as input
  pinMode(9, OUTPUT);

  pinMode(SIGNAL_LED, OUTPUT);
  pinMode(LOCK_LED, OUTPUT);

  digitalWrite(SIGNAL_LED, LOW);
  digitalWrite(LOCK_LED, LOW);

  Serial.println("Ready!");

  startLTCDecoder();
  //startLTCGenerator();
}

void loop() {
  // status led
  if(state != GENERATOR) {
    digitalWrite(SIGNAL_LED, validBitCount > 80 ? HIGH : LOW); // valid after 1 frame
    digitalWrite(LOCK_LED, state == SYNCED? HIGH : LOW);
  }

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
