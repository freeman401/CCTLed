// 141 = ~1a @ vref 1.1v
// [device]: [maxpow] [coeff] - [max current, single strand operating] [max current, both strands operating] ([combined current])
// 1: 220 100 - 1.56a 1.19a (2.38a) @ 18v
// 2: 230 100 - 1.59a 1.19a (2.37a) @ 18v
// 3: 240 130 - 1.69a 1.69a (3.38a) @ 13.3v

//Calibration values:
// MAXPOW is used to set the maximum current of the single channel
// !!! MAXPOW > (255 - coeff) !!!
// Also keep it in 200-249 range; lower - less control precision; higher - regulation loop will break
// Better to calculate a proper resistor divider at lower powers
//
// COEFF is used to adjust the color temperature transition
// < 127 - closer to constant power during entire color temperature range, less light output, more linear temperature regulation
//   127 - full power in the middle of the temperature range, maximum light output, but nonlinear regulation
// > 127 - widens the the middle 'full power' spot, even more nonlinear
#define _MAXPOW       128 //240
#define _COEFF        127 //130

#define USE_EEPROM    0    // Use EEPROM for storage of calibration values, 0 saves 42 bytes

#define _MAXPOW_ADDR  0x00 // Put your settings here with external tools
#define _COEFF_ADDR   0x01

//======================================
#define _BRI_ADDR     0x20
#define _TMP_ADDR     0x21

// IR key codes
#define IR_1          0xA2 // 1010 0010
#define IR_2          0x62 // 0110 0010
#define IR_3          0xE2 // 1110 0010
#define IR_4          0x22 // 0010 0010
#define IR_5          0x02 // 0000 0010
#define IR_6          0xC2 // 1100 0010
#define IR_7          0xE0 // 1110 0000
#define IR_8          0xA8 // 1010 1000
#define IR_9          0x90 // 1001 0000
#define IR_STAR       0x68 // 0110 1000
#define IR_0          0x98 // 1001 1000
#define IR_HASH       0xB0 // 1011 0000
#define IR_UP         0x18 // 0001 1000
#define IR_LEFT       0x10 // 0001 0000
#define IR_OK         0x38 // 0011 1000
#define IR_RIGHT      0x5A // 0101 1010
#define IR_DOWN       0x4A // 0100 1010

const PROGMEM uint8_t acmd[] = { IR_1, IR_2, IR_3, IR_4, IR_5, IR_6, IR_7, IR_8, IR_9, IR_STAR, IR_0, IR_HASH/*, IR_UP, IR_DOWN, IR_LEFT, IR_RIGHT */};
const PROGMEM uint8_t abri[] = { 31, 63, 127, 255 }; // effective values: 31, 31, 31, 63, 63, 63, 127, 127, 127, 255, 255, 255
const PROGMEM uint8_t atmp[] = { 0, 127, 255 }; // effective values: 0, 127, 255, 0, 127, 255, 0, 127, 255, 0, 127, 255

// 1/(37500Hz/3) = 80μs
#define _NEC_TOLERANCE      3   //2   // High/low tolerance, 160μs
#define _NEC_TOLERANCE2     25  //19  // Start/repeat tolerance, 1520μs
#define _NEC_SKIP_REPEAT    0   //2   // First N repeats to skip

// NEC timings
#define _NEC_HIGH_BIT       28  // 2250μs
#define _NEC_LOW_BIT        14  // 1150μs
#define _NEC_START_BIT      169 //180 // 14400μs
#define _NEC_REPEAT         141 //153 // 12300μs
#define _NEC_REPEAT_TIMEOUT 187 // 15000μs

#define _NEC_HIGH_MIN   (_NEC_HIGH_BIT - _NEC_TOLERANCE)
#define _NEC_HIGH_MAX   (_NEC_HIGH_BIT + _NEC_TOLERANCE)
#define _NEC_LOW_MIN    (_NEC_LOW_BIT - _NEC_TOLERANCE)
#define _NEC_LOW_MAX    (_NEC_LOW_BIT + _NEC_TOLERANCE)
#define _NEC_START_MIN  (_NEC_START_BIT - _NEC_TOLERANCE2)
#define _NEC_START_MAX  (_NEC_START_BIT + _NEC_TOLERANCE2)
#define _NEC_REPEAT_MIN (_NEC_REPEAT - _NEC_TOLERANCE2)
#define _NEC_REPEAT_MAX (_NEC_REPEAT + _NEC_TOLERANCE2)

#if USE_EEPROM
  #define __MAXPOW maxpow
  #define __COEFF coeff
#else
  #define __MAXPOW _MAXPOW
  #define __COEFF _COEFF
#endif

uint8_t bufferL = 0;  // Receive buffer, low byte
uint8_t bufferH = 0;  // Receive buffer, high byte
uint8_t command = 0;  // Last received command
uint8_t time = 0;     // Last received pulse width
uint8_t counter = 0;  // Received bits counter
uint8_t repeats = 0;  // Repeats counter
bool start = false;   // Receive start
bool decoded = false; // Command is received, decoded and ready to be interpreted
bool repeat = false;  // Received command is a 'repeat'

uint8_t maxpow = 0;   // Maximum single channel current
uint8_t coeff = 0;    // CCT regulation coefficent
uint8_t upd = 0;      // Skip ADC value update counter
uint8_t adc = 0;      // Averaged ADC value
uint8_t setW = 0;     // Target current setting for warm channel
uint8_t setC = 0;     // Target current setting for cold channel
uint8_t bri = 0;      // Brightness setting
uint8_t tmp = 0;      // Temperature setting
bool toff = false;    // Output is being turned off
uint8_t eet = 0;      // EEPROM save timeout

uint8_t t = 0;        // f/3 divider counter
uint8_t t1 = 0;       // f/256 divider counter

ISR(TIM0_OVF_vect) {
  if(++t > 2) {            // Divide timer frequency by 3
    t = 0;
    if(time < 255) time++; // Measure the IR pulse width
    if(++t1 == 255) {      // Divide it further for EEPROM timeout counter
      t1 = 0;
      if(eet < 255) eet++;
    }
  }
}

ISR(PCINT0_vect) {
  if(PINB & (1 << PB2)) return;             // Falling edge interrupt
  if (start && time > _NEC_LOW_MIN && time < _NEC_HIGH_MAX) {  // Check general pulse width boundaries
    uint8_t state = 2; // This not-so-elegant solution results in a more compact binary for some reason
    if (time < _NEC_LOW_MAX) state = 0;
    if (time > _NEC_HIGH_MIN) state = 1;
    if (state < 2) {                        // Valid low or high bit is received
      bufferL = (bufferL << 1) | state;     // Fill the buffer
      if(counter == 23) bufferH = bufferL;  // High byte is received, continue with low byte
      if (++counter > 31) {                 // 32 bits received
        if (bufferH & bufferL) goto exit;   // Check data integrity
        command = bufferH;                  // Valid command is received
        decoded = true;
        start = false; 
        #if _NEC_SKIP_REPEAT                // Compiler is not smart enough to optimize this
          repeats = 0; 
        #endif 
        goto exit;
      }
    }
  }
  else if (time > _NEC_REPEAT_MIN && time < _NEC_REPEAT_MAX) { // Repeat detected
    #if _NEC_SKIP_REPEAT
      if (++repeats > _NEC_SKIP_REPEAT)
    #endif
      repeat = true;
  } 
  else if (time > _NEC_START_MIN && time < _NEC_START_MAX) {   // Packet start detected
    bufferL = 0;                                               // Reset everything
    counter = 0;
    start = true;
  }
  exit: time = 0;                           // Start measuring the next pulse width
}

ISR(ADC_vect) {  
  adc = adc / 2 + ADCH / 2;                 // Crude 'averaging' technique for better accuracy
  if(TCCR0A & ((1<<WGM01)|(1<<WGM00)) && ++upd > 150 - bri / 2) { // Current regulation should work only if PWM is enabled, also skip ADC cycles for accuracy and smooth fades
    uint8_t* ocrp = (uint8_t*)&OCR0A;       // Pointer to PWM register
    uint8_t* setp = &setW;                  // Pointer to target current value
    if(ADMUX & (1<<MUX0)) {                 // Cool/warm strand channel selection
      ocrp = (uint8_t*)&OCR0B;
      setp = &setC;
    }
    if(*setp < 4) *ocrp = *setp;            // Current regulation at low brightness doesn't go all the way to zero, so it is disabled
    else if(((adc > *setp && adc - *setp > 4) || toff) && *ocrp > 0) (*ocrp)--; // Adjust PWM to match ADC value with current target value
    else if(adc < *setp && *setp - adc > 1 && *ocrp < 255) (*ocrp)++;
    ADMUX ^= (1<<MUX0);                     // Switch to the other ADC channel
    if(toff && !(OCR0A | OCR0B)) {          // If both channels are off, disable PWM to prevent slight residual glow
      toff = false;
      TCCR0A &= ~((1<<WGM01)|(1<<WGM00));
    }
    upd = 0;
  }
  ADCSRA |= (1<<ADSC);                      // Restart ADC measurement
}

uint8_t scale8(uint8_t value, uint8_t scale) {
  volatile uint8_t work = value;                // FASTLED_SCALE8_FIXED == 1; 255/255 -> 255
  //uint8_t work = 0;                         // FASTLED_SCALE8_FIXED == 0; 255/255 -> 254; Not super critical, but saves 26 bytes
  uint8_t cnt = 0x80;
  asm volatile(                             // Some assembly magic here
    "  inc %[scale]                 \n\t"   // FASTLED_SCALE8_FIXED == 1
    "  breq DONE_%=                 \n\t"   // FASTLED_SCALE8_FIXED == 1
    "  clr %[work]                  \n\t"   // FASTLED_SCALE8_FIXED == 1
    "LOOP_%=:                       \n\t"
    "  sbrc %[scale], 0             \n\t"
    "  add %[work], %[i]            \n\t"
    "  ror %[work]                  \n\t"
    "  lsr %[scale]                 \n\t"
    "  lsr %[cnt]                   \n\t"
    "brcc LOOP_%=                   \n\t"
    "DONE_%=:                       \n\t"
    : [work] "+r" (work), [cnt] "+r" (cnt)
    : [scale] "r" (scale), [i] "r" (value)
    :
  );
  return work;
}

void setup() {  
  DDRB   = (1<<PB0)|(1<<PB1); // PB0, PB1 - output
  TCCR0A = (1<<COM0A1)|(1<<COM0B1)|(1<<WGM01)|(1<<WGM00); // Fast PWM timer mode
  TCCR0B = (1<<CS00);         // No prescaler, f=37500Hz
  ADMUX  = (1<<REFS0)|(1<<ADLAR)|(1<<MUX1)|(1<<MUX0);     // ADC3 internal 1.1v reference
  ADCSRA = (1<<ADEN)|(1<<ADSC)|(1<<ADIE)|(0<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // ADC prescaler - 8, sampling rate - 1171Hz

  EEARL = _BRI_ADDR; // Set up address register
  EECR |= (1<<EERE); // Start eeprom read
  bri = EEDR;        // Restore values from EEPROM
  EEARL = _TMP_ADDR;
  EECR |= (1<<EERE);
  tmp = EEDR;
  #if USE_EEPROM
  EEARL = _MAXPOW_ADDR;
  EECR |= (1<<EERE);
  maxpow = EEDR;
  EEARL = _COEFF_ADDR;
  EECR |= (1<<EERE);
  coeff = EEDR;
  /*
  if(!(maxpow || coeff)) {       // Nice to have, but it doesn't fit
    maxpow = _MAXPOW;
    coeff = _COEFF;
    eewrite(maxpow, coeff, _MAXPOW_ADDR, _COEFF_ADDR);
  }  
  EECR  = (0<<EEPM1)|(0<<EEPM0); // Set Programming mode
  EEARL = _MAXPOW_ADDR;          // Set up address and data registers
  EEDR  = maxpow;
  EECR |= (1<<EEMPE);            // Write logical one to EEMPE
  EECR |= (1<<EEPE);             // Start eeprom write by setting EEPE
  while(EECR & (1<<EEPE));       // Wait for completion of previous write
  EEARL = _COEFF_ADDR;
  EEDR  = coeff;
  EECR |= (1<<EEMPE);
  EECR |= (1<<EEPE);
  */
  #endif

  TIMSK0 = (1<<TOIE0);  // Set up interrupts
  GIMSK  = (1<<PCIE);
  PCMSK  = (1<<PCINT2);
}

void loop() {
  if (repeat || decoded) {                  // Process the received command
    repeat = false;
    if(decoded && command == IR_OK) {
      if(TCCR0A & ((1<<WGM01)|(1<<WGM00)))  // Depending on the PWM state...
        toff = true;                        // Initiate the turn-off cycle...
      else
        TCCR0A |= (1<<WGM01)|(1<<WGM00);    // Or turn it on
    }
    else if (TCCR0A & ((1<<WGM01)|(1<<WGM00))) {     // Prevent change of settings when turned off
      if      (command == IR_UP    && bri < 251) bri += 5;
      else if (command == IR_DOWN  && bri > 4)   bri -= 5;
      else if (command == IR_LEFT  && tmp > 4)   tmp -= 5;
      else if (command == IR_RIGHT && tmp < 251) tmp += 5;
      else {
        uint8_t idx = 0;
        do {                                // Trade some speed for memory efficency
          if(command == pgm_read_byte(&acmd[idx])) { // Look-up the command in the array
            bri = pgm_read_byte(&abri[idx / 4]);     // And retrieve preset values from other arrays
            while(idx >= 3)
              idx -= 3;
            tmp = pgm_read_byte(&atmp[idx]);
            idx = 100;                      // Cheap way to exit the loop
          }
        } while(++idx < 12);        
      }
      eet = 0;                              // Reset the EEPROM write timeout counter to initiate write operation
    }
    decoded = false;
  }

  uint8_t c = __MAXPOW;                     // Calculate the cool/warm channel current for a particular color temperature
  uint8_t w = __MAXPOW;
  if(tmp < (uint8_t)~__COEFF)               // 255 - __COEFF
    c = tmp + scale8(tmp, __MAXPOW - ~__COEFF);
  if(tmp > __COEFF)
    w = ~tmp + scale8(~tmp, __MAXPOW - ~__COEFF);
  setC = scale8(c, bri);                    // Scale the target current values by brigtness
  setW = scale8(w, bri);

  if(eet < 255 && eet > 16) {               // 16 = ~330ms should be enough to ignore all but the last 'repeat' command and prevent unnecessary wear
    EECR  = (0<<EEPM1)|(0<<EEPM0);          // Set Programming mode
    EEARL = _BRI_ADDR;                      // Set up address and data registers
    EEDR  = bri;
    EECR |= (1<<EEMPE);                     // Write logical one to EEMPE
    EECR |= (1<<EEPE);                      // Start eeprom write by setting EEPE
    while(EECR & (1<<EEPE));                // Wait for completion of previous write
    EEARL = _TMP_ADDR;
    EEDR  = tmp;
    EECR |= (1<<EEMPE);
    EECR |= (1<<EEPE);
    eet = 255;                              // Ensure that timeout stays locked
  }
}