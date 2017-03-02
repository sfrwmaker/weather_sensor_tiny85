/* Humidity/Temperature sensor si7021@atmega328p-pu chip running at 1 MGz powered by the two AA batteries
 * Emulates Oregon V2.1 protocol to send the data
 * The voltage regulator ams1117-adj, supplies constant 1.24 volts to battPIN pin when is powered up through the powerPIN
 * The higher the value read on battPIN the lower the battery.
 * arduine reads 380 at 3.3 volts on battery, and 569 at 2.22 volts on the battery
*/

#include <avr/sleep.h>
#include <avr/power.h>
#include <avr/wdt.h>
#include <TinyWireM.h>
#include <USI_TWI_Master.h>
#include <SI7021.h>

const byte transPIN = 1;                              // Pin for the radio transmitter
const byte battPIN  = 3;                              // A3, Pin to read battery level
const byte ledPIN   = 4;                              // The test led pid
const uint16_t low_battery = 2100;                    // The limit for low battery (2.1 mV)

//------------ digitalWrite using direct port manipulation, attuny85 has portB only -----------------------
class directPort {
  public:
    directPort() { }
    void sendOne(void);
    void sendZero(void);
    void selectPin(byte pin) {
      set_mask = 0;
      if (pin <= 7) {
        set_mask = 1 << pin;
        clr_mask = ~set_mask;
      }
    }
  protected:
    byte set_mask;
    byte clr_mask;
  public:
    const unsigned long TIME    = 512;
    const unsigned long TWOTIME = TIME*2;
};

void directPort::sendOne(void) {
  PORTB &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TIME);
  PORTB |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TWOTIME);
  PORTB &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TIME);
}

void directPort::sendZero(void) {
  PORTB |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TIME);
  PORTB &= clr_mask;                                // digitalWrite(tx_pin, LOW);
  delayMicroseconds(TWOTIME);
  PORTB |= set_mask;                                // digitalWrite(tx_pin, HIGH);
  delayMicroseconds(TIME);
}

//----------------------------------- Send the temp & humidity using oregon v2.1 protocol -------------
class OregonSensor {
  public:
    OregonSensor(byte txPin, byte channel, byte sensorID, bool Humidity = false) {
      tx_pin = txPin;
      if (tx_pin <= 7) {                        // PORTD
        dpB.selectPin(tx_pin);
      }
      existsHumidity = Humidity;
      int type = 0xEA4C;                        // by default emulate TNHN132N
      if (existsHumidity)  type = 0x1A2D;       // emulate THGR2228N
        
      buffer[0] = type >> 8;
      buffer[1] = type & 0xFF;
      buffer[2] = channel;
      buffer[3] = sensorID;
    }
    void init(void);
    void sendTempHumidity(int temp, byte humm, bool battery);
  private:
    inline void sendOne(void)                   { dpB.sendOne(); }
    inline void sendZero(void)                  { dpB.sendZero(); }
    void sendData(const byte *data, byte size); // Send data buffer
    void sendPreamble(void);                    // Send preamble
    void sendPostamble(void);                   // Send postamble
    void sendOregon(void);                      // Send preamble, data, postamble
    void sendSync(void);
    int  sum(byte count);                       // Count the buffer summ
    void calculateAndSetChecksum(void);
    bool existsHumidity;                        // Weither THGR2228N (send Humidity)
    byte buffer[9];
    byte tx_pin;
    directPort  dpB;
    const unsigned long TIME    = 512;
    const unsigned long TWOTIME = TIME*2;
};

void OregonSensor::init(void) {
  pinMode(tx_pin, OUTPUT);
  digitalWrite(tx_pin, LOW);
}

void OregonSensor::sendData(const byte *data, byte size) {
  for(byte i = 0; i < size; ++i) {
    byte m = 1;
    byte d = data[i];
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero(); m <<= 1;
    (d & m)? sendOne(): sendZero();
  }
}

void OregonSensor::sendOregon(void) {
  sendPreamble();
  byte size = 8;
  if (existsHumidity) size = 9;
  sendData(buffer, size);
  sendPostamble();
  digitalWrite(tx_pin, LOW);
}

void OregonSensor::sendPreamble(void) {
  byte PREAMBLE[] = {0xFF,0xFF};
  sendData(PREAMBLE, 2);
}

void OregonSensor::sendPostamble(void) {
  sendZero();
  sendZero();
  sendZero();
  sendZero();
  if (!existsHumidity) return;                      // TNHN132N
  sendZero();
  sendZero();
  sendZero();
  sendZero();
}

void OregonSensor::sendSync(void) {
  byte data = 0xA;
  (data & 1)? sendOne(): sendZero(); data >>= 1;
  (data & 1)? sendOne(): sendZero(); data >>= 1;
  (data & 1)? sendOne(): sendZero(); data >>= 1;
  (data & 1)? sendOne(): sendZero();
}

int OregonSensor::sum(byte count) {
  int s = 0;
 
  for(byte i = 0; i < count; i++) {
    s += (buffer[i]&0xF0) >> 4;
    s += (buffer[i]&0xF);
  }
 
  if(int(count) != count)
    s += (buffer[count]&0xF0) >> 4;
 
  return s;
}

void OregonSensor::calculateAndSetChecksum(void) {
  if (!existsHumidity) {
    int s = ((sum(6) + (buffer[6]&0xF) - 0xa) & 0xff);
    buffer[6] |=  (s&0x0F) << 4;     buffer[7] =  (s&0xF0) >> 4;
  } else {
    buffer[8] = ((sum(8) - 0xa) & 0xFF);
  }
}

void OregonSensor::sendTempHumidity(int temp, byte humm, bool battery) {  // temperature centegrees * 10
  if(!battery) buffer[4] = 0x0C; else buffer[4] = 0x00;

  if(temp < 0) {
    buffer[6] = 0x08;
    temp *= -1; 
  } else {
    buffer[6] = 0x00;
  }
  byte d3 = temp % 10;                              // Set temperature decimal part
  buffer[4] |= d3 << 4;
  temp /= 10;
  byte d1 = temp / 10;                              // 1st decimal digit of the temperature
  byte d2 = temp % 10;                              // 2nd deciaml digit of the temperature
  buffer[5] = d1 << 4;
  buffer[5] |= d2;

  if (existsHumidity) {                             // THGR2228N
    buffer[7] = humm / 10;
    buffer[6] |= (humm % 10) << 4;
  }
  calculateAndSetChecksum();

  sendOregon();                                     // The v2.1 protocol send the message two times
  delayMicroseconds(TWOTIME*8);
  sendOregon();
}
//================================ End of the class definitions ==================================================

SI7021 sensor;
OregonSensor os(transPIN, 0x20, 0xAA, true);

void enterSleep(void) {
  ADCSRA &= ~(1<<ADEN);                               // disable ADC
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);                // the lowest power consumption
  sleep_enable();
  sleep_cpu();                                        // Now enter sleep mode.
  
  // The program will continue from here after the WDT timeout. 
  sleep_disable();                                    // First thing to do is disable sleep.
  power_all_enable();                                 // Re-enable the peripherals.
  ADCSRA |= 1<<ADEN;                                  // enable ADC
}

/*
 * https://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/
 * Read 1.1V reference against AVcc
 * set the reference to Vcc and the measurement to the internal 1.1V reference
 */
uint32_t readVcc() {                                  // Vcc in millivolts
  #if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
    ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
    ADMUX = _BV(MUX5) | _BV(MUX0);
  #elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
    ADMUX = _BV(MUX3) | _BV(MUX2);
  #else
    ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
  #endif  

  delay(2);                                           // Wait for Vref to settle
  ADCSRA |= _BV(ADSC);                                // Start conversion
  while (bit_is_set(ADCSRA,ADSC));                    // measuring

  uint8_t low  = ADCL;                                // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH;                                // unlocks both

  long result = (high<<8) | low;

  result = 1125300L / result;                         // Calculate Vcc (in mV); 112530 = 1.1*1023*1000
  return result;
}

volatile byte f_wdt = 1;
 
 void setup() {
  pinMode(battPIN,  INPUT);
  pinMode(ledPIN, OUTPUT);
  digitalWrite(ledPIN, LOW);                          // Switch off the led
  os.init();
  delay(15000);                                       // This timeout allows to flash new program
  
  // Setup the WDT
  MCUSR &= ~(1<<WDRF);                                // Clear the reset flag
  
  // In order to change WDE or the prescaler, we need to
  // set WDCE (This will allow updates for 4 clock cycles).
  WDTCR |= (1<<WDCE) | (1<<WDE);
  WDTCR = 1<<WDP0 | 1<<WDP3;                         // set new watchdog timeout prescaler value 8.0 seconds
  WDTCR |= _BV(WDIE);                                // Enable the WD interrupt (note no reset).
 
  sensor.begin();
}
 
 void loop() {
  static int awake_counter = 1;
  static int check_battery = 0;
  static bool batteryOK = true;

  if (f_wdt == 1) {
    f_wdt = 0;
    --awake_counter;
    if (awake_counter <= 0) {                         // Send Weather data
      awake_counter = 5;
      // Check the battery level every 100 wake ups 100*5*8s = 4000 seconds

      if (batteryOK && (check_battery == 0)) {        // The battery cannot repare by itself!
        uint32_t mV = readVcc();
        batteryOK = (mV >= low_battery);
      }
      ++check_battery; if (check_battery > 100) check_battery = 0;

      int temperature = sensor.getCelsiusHundredths();
      if (temperature > 0)
        temperature += 5;
      else
        temperature -= 5;
      temperature /= 10;
      byte humidity   = sensor.getHumidityPercent();

      digitalWrite(ledPIN, HIGH);                     // turn-on the led during the data transmition
      os.sendTempHumidity(temperature, humidity, batteryOK);
      digitalWrite(ledPIN, LOW);
    }
    enterSleep();                                     // power-save mode for 8 seconds
  }

}

ISR(WDT_vect) {
  if (f_wdt == 0) f_wdt = 1;
}


