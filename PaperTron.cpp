#include "PaperTron.h"
#include "wiring_private.h"
#include "pins_arduino.h"

PaperTron::PaperTron() {}

void PaperTron::Begin(){
  Serial.begin(9600);
  Serial.println("Hi! I'm PaperTron. You can use my Digital and Analog pins.");
  
}

void PaperTron::touchBegin(){
  if (!begin(0x5A)) {
  Serial.println("Error! TOUCH pins not found. Please check wiring.");
  while (1);
  }
  Serial.println("Hurray! Now you can use my TOUCH pads.");

}  

void PaperTron::Blink(int pin, int delayTime){
  if(pin==4 || pin==6){Serial.println("Sorry! Pin 4 and 6 are not available. Please use any other pin. "); delay(1000*delayTime);}
  else{
  digitalWrite(pin,HIGH);
  delay(delayTime);
  digitalWrite(pin,LOW);
  delay(delayTime);}
  }

void PaperTron::fadeIn(int pin, int delayTime){
  if(pin==4 || pin==6 || pin==2 || pin==7 || pin==8 || pin==13){
  Serial.println("Sorry! This pin is not available. Please use a PWM Pin."); 
  Serial.println("PWM Pins are 3,5,9,10,11,12.");delay(10000*delayTime);}
  else{
  for (int i = 0; i < 255; i++){ analogWrite(pin, i); delay(delayTime); }
  }
}

void PaperTron::fadeOut(int pin, int delayTime){
  if(pin==4 || pin==6 ||pin==2 || pin==7 || pin==8 || pin==13){
  Serial.println("Sorry! This pin is not available. Please use a PWM Pin."); 
  Serial.println("PWM Pins are 3,5,9,10,11,12.");delay(10000*delayTime);}
  else{
  for (int i = 255; i > 0; i--){ analogWrite(pin, i); delay(delayTime); }
  }
}

String PaperTron::IR(int irPin){
    if(irPin==6 || irPin==7 || irPin==8 || irPin==9){
  Serial.println("Sorry! This pin is not available. Please use any other Pin."); 
  Serial.println("Pins which can be used: 2,3,5,10,11,12,13.");delay(100000);}
  else{
        if(digitalRead(irPin)==HIGH){Serial.println("IR: ON"); delay(500); return "ON";}
        else {Serial.println("IR: OFF"); delay(500); return "OFF";}
  }  
}

int PaperTron::Ultrasonic(int trigPin,int echoPin){
  if(trigPin||echoPin==4 || trigPin||echoPin==6 || echoPin||trigPin==2 || trigPin||echoPin==7 || trigPin||echoPin==8 || trigPin||echoPin==13 || trigPin==9){
  Serial.println("Sorry! These pins are not available. Please use a PWM Pin."); 
  Serial.println("PWM Pins for Ultrasonic are 3,5,10,11,12.");delay(100000);}
  else{
digitalWrite(trigPin, LOW);
delayMicroseconds(2);
digitalWrite(trigPin, HIGH);
delayMicroseconds(10);
digitalWrite(trigPin, LOW);
long duration = pulseIn(echoPin, HIGH);
int distance= duration*0.034/2;
  Serial.print("Distance: ");
  Serial.println(distance);
  delay(500);
  return distance;
  }
}

int PaperTron::LDR(int LDRPin){
  int ldrStatus = analogRead(LDRPin);
  Serial.print("LDR Value: ");
  Serial.println(ldrStatus);
  delay(500);
  return ldrStatus;
  }

String PaperTron::Tilt(int tiltPin){
    if(tiltPin==6 || tiltPin==7 || tiltPin==8 || tiltPin==9){
  Serial.println("Sorry! This pin is not available. Please use any other Pin."); 
  Serial.println("Pins which can be used: 2,3,5,10,11,12,13.");delay(100000);}
  else{
        if(digitalRead(tiltPin)==HIGH){Serial.println("Tilted"); delay(500); return "Tilted";}
        else {Serial.println("Not Tilted"); delay(500); return "Not Tilted";}
  }  
}

String PaperTron::Reed(int reedPin){
    if(reedPin==6 || reedPin==7 || reedPin==8 || reedPin==9){
  Serial.println("Sorry! This pin is not available. Please use any other Pin."); 
  Serial.println("Pins which can be used: 2,3,5,10,11,12,13.");delay(100000);}
  else{
        if(digitalRead(reedPin)==HIGH){Serial.println("ON"); delay(500); return "ON";}
        else {Serial.println("OFF"); delay(500); return "OFF";}
  }  
}
  
float PaperTron::Temperature(int tempPin){
 int  val = analogRead(tempPin);
float mv  = ( val/1024.0)*5000;
float cel = mv/10;
float farh = (cel*9)/5 + 32;

Serial.print("Temperature: ");
Serial.print(cel);
Serial.print("*C");
Serial.println();
delay(500);
return cel;

/* uncomment this to get temperature in farenhite
Serial.print("TEMPRATURE = ");
Serial.print(farh);
Serial.print("*F");
Serial.println();
*/
  
  }


void PaperTron::pinModeRGB(int PinR, int PinG, int PinB) {
pinMode(PinR,OUTPUT);
pinMode(PinG,OUTPUT);
pinMode(PinB,OUTPUT);
}

void PaperTron::RGB(int PinR, int PinG, int PinB, int red, int green, int blue){
  analogWrite(PinR, red);
  analogWrite(PinG, green);
  analogWrite(PinB, blue);
}

void PaperTron::pinModeOnBoardRGB() {
pinMode(9,OUTPUT);
pinMode(8,OUTPUT);
pinMode(7,OUTPUT);
}

void PaperTron::onBoardRGB( int red, int green, int blue){
  analogWrite(9, red);
  analogWrite(8, green);
  analogWrite(7, blue);
}



void PaperTron::RGBDisco(int Delay) {
  onBoardRGB( 255, 0, 0); // Red
  delay(Delay);
  onBoardRGB(0, 255, 0); // Green
  delay(Delay);
  onBoardRGB(0, 0, 255); // Blue
  delay(Delay);
  onBoardRGB(255, 255, 125); // Raspberry
  delay(Delay);
  onBoardRGB(0, 255, 255); // Cyan
  delay(Delay);
  onBoardRGB(255, 0, 255); // Magenta
  delay(Delay);
  onBoardRGB(255, 255, 0); // Yellow
  delay(Delay);
  onBoardRGB(255, 255, 255); // White
  delay(Delay);
}


void PaperTron::RGBColorChange(int Delay) {

counter++;
Serial.println(counter);
   switch(counter) 
        { 
         case 1: onBoardRGB(255, 0, 0); // Red
           delay(Delay);
          break;
            
         case 2:onBoardRGB(150,10,203);//PINK
          delay(Delay);
          break;
            
          case 3: onBoardRGB(0, 255, 0);//GREEN
            delay(Delay);
          break;
            
          case 4:   onBoardRGB(0, 0, 255);//BLUE
            delay(Delay);
          break;
            
          case 5:  onBoardRGB(255, 255, 125);//RASBERRY
            delay(Delay);
          break; 
            
          case 6:   onBoardRGB(0, 255, 255); // Cyan
          delay(Delay);
          break; 
            
          case 7:  onBoardRGB(255, 0, 255); // Magenta
            delay(Delay);
          break;
            
          case 8: onBoardRGB(255, 255, 0); // Yellow
            delay(Delay);   
          break;  
           
          case 9: onBoardRGB(255, 255, 255); // Whitergb(153,50,204)
            delay(Delay);   
          break;  

          case 10: onBoardRGB(153,50,204); // Whitergb(153,50,204)
            delay(Delay);   
          break;
             
          case 11: onBoardRGB(34,139,34); // Whitergb(153,50,204) 75,0,130)
            delay(Delay);   
          break;
        }
         
     if(counter==11) counter = 0;  
}


void PaperTron::touchPads() {
  currtouched = touched();
  for (i=0; i<12; i++) {
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ) {
      Serial.print("Pad ");Serial.print(i); Serial.println(" Touched");
    }

    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      Serial.print("Pad ");Serial.print(i); Serial.println(" Released");
    }
  }
  lasttouched = currtouched;
}

uint8_t PaperTron::touchRead(uint8_t touchpin) {
  currtouched = touched();
  for (i=0; i<12; i++) {
    if ((currtouched & _BV(i)) && !(lasttouched & _BV(i)) ){
      if(i==touchpin){Serial.print("Pad ");Serial.print(i); Serial.println(" HIGH");}
    }
    
    if (!(currtouched & _BV(i)) && (lasttouched & _BV(i)) ) {
      if(i==touchpin){Serial.print("Pad ");Serial.print(i); Serial.println(" LOW"); }
    }
  }
  lasttouched = currtouched;
 }

bool PaperTron::isHigh(uint8_t pin) {
  curr = touched();
 if(pin==0&&curr==1 || pin==1&&curr==2 ||  pin==2&&curr==4 || pin==3&&curr==8 || pin==4&&curr==16 || pin==5&&curr==32 || pin==6&&curr==64 || pin==7&&curr==128 || pin==8&&curr==256 || pin==9&&curr==512 || pin==10&&curr==1024 || pin==11&&curr==2048)
 {return true;}
 else return false;

 }


boolean PaperTron::begin(uint8_t i2caddr, TwoWire *theWire,uint8_t touchThreshold,uint8_t releaseThreshold) {

  _i2caddr = i2caddr;
  _wire = theWire;

  _wire->begin();

  // soft reset
  writeRegister(MPR121_SOFTRESET, 0x63);
  delay(1);
  for (uint8_t i = 0; i < 0x7F; i++) {
  }

  writeRegister(MPR121_ECR, 0x0);

  uint8_t c = readRegister8(MPR121_CONFIG2);

  if (c != 0x24)
    return false;

  setThresholds(touchThreshold, releaseThreshold);
  writeRegister(MPR121_MHDR, 0x01);
  writeRegister(MPR121_NHDR, 0x01);
  writeRegister(MPR121_NCLR, 0x0E);
  writeRegister(MPR121_FDLR, 0x00);

  writeRegister(MPR121_MHDF, 0x01);
  writeRegister(MPR121_NHDF, 0x05);
  writeRegister(MPR121_NCLF, 0x01);
  writeRegister(MPR121_FDLF, 0x00);

  writeRegister(MPR121_NHDT, 0x00);
  writeRegister(MPR121_NCLT, 0x00);
  writeRegister(MPR121_FDLT, 0x00);

  writeRegister(MPR121_DEBOUNCE, 0);
  writeRegister(MPR121_CONFIG1, 0x10); // default, 16uA charge current
  writeRegister(MPR121_CONFIG2, 0x20); // 0.5uS encoding, 1ms period

#ifdef AUTOCONFIG
  writeRegister(MPR121_AUTOCONFIG0, 0x0B);

  // correct values for Vdd = 3.3V
  writeRegister(MPR121_UPLIMIT, 200);     // ((Vdd - 0.7)/Vdd) * 256
  writeRegister(MPR121_TARGETLIMIT, 180); // UPLIMIT * 0.9
  writeRegister(MPR121_LOWLIMIT, 130);    // UPLIMIT * 0.65
#endif

  // enable X electrodes and start MPR121
  byte ECR_SETTING =
      B10000000 + 12; // 5 bits for baseline tracking & proximity disabled + X
                      // amount of electrodes running (12)
  writeRegister(MPR121_ECR, ECR_SETTING); // start with above ECR setting

  return true;
}

void PaperTron::setThreshholds(uint8_t touch, uint8_t release) {
  setThresholds(touch, release);
}

void PaperTron::setThresholds(uint8_t touch, uint8_t release) {
  // first stop sensor to make changes
  writeRegister(MPR121_ECR, 0x00);
  // set all thresholds (the same)
  for (uint8_t i = 0; i < 12; i++) {
    writeRegister(MPR121_TOUCHTH_0 + 2 * i, touch);
    writeRegister(MPR121_RELEASETH_0 + 2 * i, release);
  }
  // turn the sensor on again
  writeRegister(MPR121_ECR, 0x8F);
}

uint16_t PaperTron::filteredData(uint8_t t) {
  if (t > 12)
    return 0;
  return readRegister16(MPR121_FILTDATA_0L + t * 2);
}

uint16_t PaperTron::baselineData(uint8_t t) {
  if (t > 12)
    return 0;
  uint16_t bl = readRegister8(MPR121_BASELINE_0 + t);
  return (bl << 2);
}

uint16_t PaperTron::touched(void) {
  uint16_t t = readRegister16(MPR121_TOUCHSTATUS_L);
  return t & 0x0FFF;
}

uint8_t PaperTron::readRegister8(uint8_t reg) {
  _wire->beginTransmission(_i2caddr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_i2caddr, 1);
  if (_wire->available() < 1)
    return 0;
  return (_wire->read());
}

uint16_t PaperTron::readRegister16(uint8_t reg) {
  _wire->beginTransmission(_i2caddr);
  _wire->write(reg);
  _wire->endTransmission(false);
  _wire->requestFrom(_i2caddr, 2);
  if (_wire->available() < 2)
    return 0;
  uint16_t v = _wire->read();
  v |= ((uint16_t)_wire->read()) << 8;
  return v;
}

void PaperTron::writeRegister(uint8_t reg, uint8_t value) {
  // MPR121 must be put in Stop Mode to write to most registers
  bool stop_required = true;
  uint8_t ECR = readRegister8(
      MPR121_ECR); // first get the current set value of the MPR121_ECR register
  if (reg == MPR121_ECR || (0x73 <= reg && reg <= 0x7A)) {
    stop_required = false;
  }
  if (stop_required) {
    _wire->beginTransmission(_i2caddr);
    _wire->write(MPR121_ECR);
    _wire->write(0x00); // clear this register to set stop modus
    _wire->endTransmission();
  }

  _wire->beginTransmission(_i2caddr);
  _wire->write((uint8_t)reg);
  _wire->write((uint8_t)(value));
  _wire->endTransmission();

  if (stop_required) {
    _wire->beginTransmission(_i2caddr);
    _wire->write(MPR121_ECR);
    _wire->write(ECR); // write back the previous set ECR settings
    _wire->endTransmission();
  }
}



void PaperTron::pinMode(uint8_t pin, uint8_t mode){
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *reg, *out;

  if (port == NOT_A_PIN) return;

  // JWS: can I let the optimizer do this?
  reg = portModeRegister(port);
  out = portOutputRegister(port);

  if (mode == INPUT) { 
    uint8_t oldSREG = SREG;
                cli();
    *reg &= ~bit;
    *out &= ~bit;
    SREG = oldSREG;
  } else if (mode == INPUT_PULLUP) {
    uint8_t oldSREG = SREG;
                cli();
    *reg &= ~bit;
    *out |= bit;
    SREG = oldSREG;
  } else {
    uint8_t oldSREG = SREG;
                cli();
    *reg |= bit;
    SREG = oldSREG;
  }
}

static void PaperTron::turnOffPWM(uint8_t timer){
  switch (timer)
  {
    #if defined(TCCR1A) && defined(COM1A1)
    case TIMER1A:   cbi(TCCR1A, COM1A1);    break;
    #endif
    #if defined(TCCR1A) && defined(COM1B1)
    case TIMER1B:   cbi(TCCR1A, COM1B1);    break;
    #endif
    #if defined(TCCR1A) && defined(COM1C1)
    case TIMER1C:   cbi(TCCR1A, COM1C1);    break;
    #endif
    
    #if defined(TCCR2) && defined(COM21)
    case  TIMER2:   cbi(TCCR2, COM21);      break;
    #endif
    
    #if defined(TCCR0A) && defined(COM0A1)
    case  TIMER0A:  cbi(TCCR0A, COM0A1);    break;
    #endif
    
    #if defined(TCCR0A) && defined(COM0B1)
    case  TIMER0B:  cbi(TCCR0A, COM0B1);    break;
    #endif
    #if defined(TCCR2A) && defined(COM2A1)
    case  TIMER2A:  cbi(TCCR2A, COM2A1);    break;
    #endif
    #if defined(TCCR2A) && defined(COM2B1)
    case  TIMER2B:  cbi(TCCR2A, COM2B1);    break;
    #endif
    
    #if defined(TCCR3A) && defined(COM3A1)
    case  TIMER3A:  cbi(TCCR3A, COM3A1);    break;
    #endif
    #if defined(TCCR3A) && defined(COM3B1)
    case  TIMER3B:  cbi(TCCR3A, COM3B1);    break;
    #endif
    #if defined(TCCR3A) && defined(COM3C1)
    case  TIMER3C:  cbi(TCCR3A, COM3C1);    break;
    #endif

    #if defined(TCCR4A) && defined(COM4A1)
    case  TIMER4A:  cbi(TCCR4A, COM4A1);    break;
    #endif          
    #if defined(TCCR4A) && defined(COM4B1)
    case  TIMER4B:  cbi(TCCR4A, COM4B1);    break;
    #endif
    #if defined(TCCR4A) && defined(COM4C1)
    case  TIMER4C:  cbi(TCCR4A, COM4C1);    break;
    #endif      
    #if defined(TCCR4C) && defined(COM4D1)
    case TIMER4D: cbi(TCCR4C, COM4D1);  break;
    #endif      
      
    #if defined(TCCR5A)
    case  TIMER5A:  cbi(TCCR5A, COM5A1);    break;
    case  TIMER5B:  cbi(TCCR5A, COM5B1);    break;
    case  TIMER5C:  cbi(TCCR5A, COM5C1);    break;
    #endif
  }
}

void PaperTron::digitalWrite(uint8_t pin, uint8_t val){
  uint8_t timer = digitalPinToTimer(pin);
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);
  volatile uint8_t *out;

  if (port == NOT_A_PIN) return;
  if (timer != NOT_ON_TIMER) turnOffPWM(timer);

  out = portOutputRegister(port);

  uint8_t oldSREG = SREG;
  cli();

  if (val == LOW) {
    *out &= ~bit;
  } else {
    *out |= bit;
  }

  SREG = oldSREG;
}

int PaperTron::digitalRead(uint8_t pin){
  uint8_t timer = digitalPinToTimer(pin);
  uint8_t bit = digitalPinToBitMask(pin);
  uint8_t port = digitalPinToPort(pin);

  if (port == NOT_A_PIN) return LOW;

  // If the pin that support PWM output, we need to turn it off
  // before getting a digital reading.
  if (timer != NOT_ON_TIMER) turnOffPWM(timer);

  if (*portInputRegister(port) & bit) return HIGH;
  return LOW;
}

void PaperTron::analogReference(uint8_t mode){
  analog_reference = mode;
}

int PaperTron::analogRead(uint8_t pin){
  uint8_t low, high;

#if defined(analogPinToChannel)
#if defined(__AVR_ATmega32U4__)
  if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#endif
  pin = analogPinToChannel(pin);
#elif defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  if (pin >= 54) pin -= 54; // allow for channel or pin numbers
#elif defined(__AVR_ATmega32U4__)
  if (pin >= 18) pin -= 18; // allow for channel or pin numbers
#elif defined(__AVR_ATmega1284__) || defined(__AVR_ATmega1284P__) || defined(__AVR_ATmega644__) || defined(__AVR_ATmega644A__) || defined(__AVR_ATmega644P__) || defined(__AVR_ATmega644PA__)
  if (pin >= 24) pin -= 24; // allow for channel or pin numbers
#else
  if (pin >= 14) pin -= 14; // allow for channel or pin numbers
#endif

#if defined(ADCSRB) && defined(MUX5)
  ADCSRB = (ADCSRB & ~(1 << MUX5)) | (((pin >> 3) & 0x01) << MUX5);
#endif

#if defined(ADMUX)
#if defined(__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = (analog_reference << 4) | (pin & 0x07);
#else
  ADMUX = (analog_reference << 6) | (pin & 0x07);
#endif
#endif


#if defined(ADCSRA) && defined(ADCL)
  sbi(ADCSRA, ADSC);
  while (bit_is_set(ADCSRA, ADSC));
  low  = ADCL;
  high = ADCH;
#else
  // we dont have an ADC, return 0
  low  = 0;
  high = 0;
#endif
  return (high << 8) | low;
}

void PaperTron::analogWrite(uint8_t pin, int val){
  pinMode(pin, OUTPUT);
  if (val == 0)
  {
    digitalWrite(pin, LOW);
  }
  else if (val == 255)
  {
    digitalWrite(pin, HIGH);
  }
  else
  {
    switch(digitalPinToTimer(pin))
    {
      // XXX fix needed for atmega8
      #if defined(TCCR0) && defined(COM00) && !defined(__AVR_ATmega8__)
      case TIMER0A:
        // connect pwm to pin on timer 0
        sbi(TCCR0, COM00);
        OCR0 = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR0A) && defined(COM0A1)
      case TIMER0A:
        // connect pwm to pin on timer 0, channel A
        sbi(TCCR0A, COM0A1);
        OCR0A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR0A) && defined(COM0B1)
      case TIMER0B:
        // connect pwm to pin on timer 0, channel B
        sbi(TCCR0A, COM0B1);
        OCR0B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR1A) && defined(COM1A1)
      case TIMER1A:
        // connect pwm to pin on timer 1, channel A
        sbi(TCCR1A, COM1A1);
        OCR1A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR1A) && defined(COM1B1)
      case TIMER1B:
        // connect pwm to pin on timer 1, channel B
        sbi(TCCR1A, COM1B1);
        OCR1B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR1A) && defined(COM1C1)
      case TIMER1C:
        // connect pwm to pin on timer 1, channel B
        sbi(TCCR1A, COM1C1);
        OCR1C = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR2) && defined(COM21)
      case TIMER2:
        // connect pwm to pin on timer 2
        sbi(TCCR2, COM21);
        OCR2 = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR2A) && defined(COM2A1)
      case TIMER2A:
        // connect pwm to pin on timer 2, channel A
        sbi(TCCR2A, COM2A1);
        OCR2A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR2A) && defined(COM2B1)
      case TIMER2B:
        // connect pwm to pin on timer 2, channel B
        sbi(TCCR2A, COM2B1);
        OCR2B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR3A) && defined(COM3A1)
      case TIMER3A:
        // connect pwm to pin on timer 3, channel A
        sbi(TCCR3A, COM3A1);
        OCR3A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR3A) && defined(COM3B1)
      case TIMER3B:
        // connect pwm to pin on timer 3, channel B
        sbi(TCCR3A, COM3B1);
        OCR3B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR3A) && defined(COM3C1)
      case TIMER3C:
        // connect pwm to pin on timer 3, channel C
        sbi(TCCR3A, COM3C1);
        OCR3C = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR4A)
      case TIMER4A:
        //connect pwm to pin on timer 4, channel A
        sbi(TCCR4A, COM4A1);
        #if defined(COM4A0)   // only used on 32U4
        cbi(TCCR4A, COM4A0);
        #endif
        OCR4A = val;  // set pwm duty
        break;
      #endif
      
      #if defined(TCCR4A) && defined(COM4B1)
      case TIMER4B:
        // connect pwm to pin on timer 4, channel B
        sbi(TCCR4A, COM4B1);
        OCR4B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR4A) && defined(COM4C1)
      case TIMER4C:
        // connect pwm to pin on timer 4, channel C
        sbi(TCCR4A, COM4C1);
        OCR4C = val; // set pwm duty
        break;
      #endif
        
      #if defined(TCCR4C) && defined(COM4D1)
      case TIMER4D:       
        // connect pwm to pin on timer 4, channel D
        sbi(TCCR4C, COM4D1);
        #if defined(COM4D0)   // only used on 32U4
        cbi(TCCR4C, COM4D0);
        #endif
        OCR4D = val;  // set pwm duty
        break;
      #endif

              
      #if defined(TCCR5A) && defined(COM5A1)
      case TIMER5A:
        // connect pwm to pin on timer 5, channel A
        sbi(TCCR5A, COM5A1);
        OCR5A = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR5A) && defined(COM5B1)
      case TIMER5B:
        // connect pwm to pin on timer 5, channel B
        sbi(TCCR5A, COM5B1);
        OCR5B = val; // set pwm duty
        break;
      #endif

      #if defined(TCCR5A) && defined(COM5C1)
      case TIMER5C:
        // connect pwm to pin on timer 5, channel C
        sbi(TCCR5A, COM5C1);
        OCR5C = val; // set pwm duty
        break;
      #endif

      case NOT_ON_TIMER:
      default:
        if (val < 128) {
          digitalWrite(pin, LOW);
        } else {
          digitalWrite(pin, HIGH);
        }
    }
  }
}
