/*
 * Fridge Alarm: Beep when detect (fridge) light - long stay battery life (about 3/4 years)
 * Copyright 2014 Team RCFree44 
 */

#include <avr/io.h>
#include <avr/sleep.h>    // Sleep Modes
#include <avr/power.h>    // Power management
#include <avr/wdt.h>      // Watchdog timer

typedef unsigned char  U8;
typedef unsigned short U16;
typedef unsigned long  U32;

#define MOD_DIN  0
#define LDR_OUT  1
#define LDR_AIN  A1
#define BUZ_OUT  3
#define LED_OUT  4

#define ACTIV_TEMP_SENSOR  0
#define ACTIV_TEST_MODE    0

#define TEMP_OFFSET    4
#define TEMP_MIN       2
#define TEMP_MAX       8
#define LDR_THRESHOLD  768

// -----------------------------------------------------------//
// ------- Global  -------------------------------------------// 
// -----------------------------------------------------------//

#if ACTIV_TEMP_SENSOR == 1
  byte Initial_ADCSRA_Value;
  byte Initial_ADMUX_Value;
  signed short Temp_Value;
#endif

bool Current_Mode;

// -----------------------------------------------------------//
// ------- Watchdog / Sleep ----------------------------------// 
// -----------------------------------------------------------//

enum 
{
  WDT_16_MS  =  0b000000,
  WDT_32_MS  =  0b000001,
  WDT_64_MS  =  0b000010,
  WDT_128_MS =  0b000011,
  WDT_256_MS =  0b000100,
  WDT_512_MS =  0b000101,
  WDT_1_SEC  =  0b000110,
  WDT_2_SEC  =  0b000111,
  WDT_4_SEC  =  0b100000,
  WDT_8_SEC  =  0b100001,
};  // end of WDT intervals enum

// WDG sleep 
void WDG_Sleep(const byte interval)
{
  noInterrupts(); 
  
  // .......
  // preparing the WDG
 
  // clear various "reset" flags
  MCUSR = 0;     
  // allow changes, disable reset
  WDTCR = _BV (WDCE) | _BV (WDE);
  // set interrupt mode and an interval 
  WDTCR = _BV (WDIE) | interval;    // set WDIE, and requested delay
  // pat the dog
  wdt_reset();  

  // .......
  // entering WDG sleep mode 
  
  // disable ADC to save power
  byte old_ADCSRA = ADCSRA;
  ADCSRA = 0;  
  
  // power off ADC, Timer 0 and 1, serial interface
  power_all_disable ();
  
  // define strong sleep mode (wake-up only per WDG or some IO)
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();

  // turn off brown-out enable in software
  // this must be done within 4 clock cycles of above
//MCUCR = bit (BODS) | bit (BODSE);  // one cycle more
//MCUCR = bit (BODS);                // one cycle more 
  interrupts ();                     // one cycle more
  sleep_cpu ();                      // one cycle more: just entering within 4 clock cycles

  // .......
  // exiting WDG sleep mode

  // cancel sleep as a precaution
  sleep_disable(); 

  // power everything back on
  power_all_enable ();   

  // re-enable ADC conversion
  ADCSRA = old_ADCSRA;                   
} 

// Watchdog Interrupt Service / is executed when watchdog timed out
ISR(WDT_vect) 
{
  // disable watchdog
  wdt_disable();
}

// -----------------------------------------------------------//
// ------- Read Sensor ---------------------------------------// 
// -----------------------------------------------------------//
#if ACTIV_TEMP_SENSOR == 1
signed short Read_Internal_Temp() 
{
  // analogReference( INTERNAL1V1 );
  // ATTiny85 datasheet p140 (17.13.2), p137 (17.12)
  // Configure ADMUX
  ADMUX = B1111;                // Select temperature sensor
  ADMUX &= ~_BV( ADLAR );       // Right-adjust result
  ADMUX |= _BV( REFS1 );                      // Set Ref voltage
  ADMUX &= ~( _BV( REFS0 ) | _BV( REFS2 ) );  // to 1.1V

  // Configure ADCSRA
  ADCSRA = Initial_ADCSRA_Value;            // re-start & enable ADC
  ADCSRA &= ~( _BV( ADATE ) |_BV( ADIE ) ); // Disable autotrigger, Disable Interrupt
  ADCSRA |= _BV(ADEN);                      // Enable ADC
  ADCSRA |= _BV(ADSC);                      // Start first conversion
  
  // Start & Wait conversion
  ADCSRA |= (1 << ADSC);  //Start temperature conversion
  while (bit_is_set(ADCSRA, ADSC));  
  
  // Read internal temperature
  byte low  = ADCL;
  byte high = ADCH;
  signed short temperature = (high << 8) | low;
  // Convert Kelvin in Â°C + apply offset
  temperature -= 273 + TEMP_OFFSET;
  
  // restoring register
  ADMUX  = Initial_ADMUX_Value;
  ADCSRA = Initial_ADCSRA_Value;
  
  // return temperature
  return temperature;
}
#endif //  ACTIV_TEMP_SENSOR

signed short Read_LDR() 
{
  // power LDR
  digitalWrite(LDR_OUT, HIGH);
  signed short value = analogRead(LDR_AIN);
#if ACTIV_TEST_MODE == 0
  digitalWrite(LDR_OUT, LOW);
#endif
  return value;
}

// -----------------------------------------------------------//
// ------- LowPower ------------------------------------------// 
// -----------------------------------------------------------//

void Low_Power(const byte interval)
{
#if ACTIV_TEST_MODE == 0
  // Set all IO to input "low" (not "pullup") to reduce comsumption
  pinMode(MOD_DIN, INPUT); digitalWrite (MOD_DIN, LOW); 
  pinMode(LDR_OUT, INPUT); digitalWrite (LDR_OUT, LOW); 
  pinMode(BUZ_OUT, INPUT); digitalWrite (BUZ_OUT, LOW); 
  pinMode(LED_OUT, INPUT); digitalWrite (LED_OUT, LOW); 
  
  // sleep / delay
  WDG_Sleep(interval);
#else // ACTIV_TEST_MODE
  delay(1000);
#endif

#if ACTIV_TEMP_SENSOR == 1
  // Read Temperature
  Temp_Value = Read_Internal_Temp();
#endif // ACTIV_TEMP_SENSOR

  // Set IO in operation
  pinMode(MOD_DIN, INPUT_PULLUP);
  pinMode(LDR_OUT, OUTPUT);
  pinMode(BUZ_OUT, OUTPUT);
  pinMode(LED_OUT, OUTPUT);
}

// -----------------------------------------------------------//
// ------- Debug ---------------------------------------------// 
// -----------------------------------------------------------//

void DebugPrintLed(unsigned short value)
{
  unsigned short bit_mask = 1 << 15;
  for(int i=0; i<16; ++i, bit_mask >>=1)
  {
    if (value < bit_mask) continue;
    if (value & bit_mask)
    {
      // ONE
      digitalWrite(LED_OUT, HIGH); digitalWrite(BUZ_OUT, HIGH);
      delay(500);
    }
    else
    {
      // ZERO
      digitalWrite(LED_OUT, HIGH); digitalWrite(BUZ_OUT, HIGH);
      delay(100);
      digitalWrite(LED_OUT, LOW);  digitalWrite(BUZ_OUT, LOW);
      delay(300);
      digitalWrite(LED_OUT, HIGH); digitalWrite(BUZ_OUT, HIGH);
      delay(100);
    }
    digitalWrite(LED_OUT, LOW);    digitalWrite(BUZ_OUT, LOW);
    delay(500);    
  }
}

// -----------------------------------------------------------//
// ------- Alarm ---------------------------------------------// 
// -----------------------------------------------------------//

bool Alarm_Test()
{
  // Check Temperature
#if ACTIV_TEMP_SENSOR == 1
  if (0 == digitalRead(MOD_DIN))
  {
    signed short Temp = Read_Internal_Temp(); 
    if (Temp < TEMP_MIN || Temp > TEMP_MAX)
    {
      return true;
    }
  }
#endif // ACTIV_TEMP_SENSOR

  // Check LDR
  if (Read_LDR() < LDR_THRESHOLD)
  {
    return true;
  }
  
  // No Trouble detected
  return false;
} 

bool Check_Confirmed()
{
  // Do 10 detections ~ 10 ms
  U8 nb = 0;
  for(U8 loop=0; loop<10; ++loop)
  {
    //delayMicroseconds(1000);
    if (Alarm_Test()) ++nb;
  }
  
  // Check Threshold
  if (nb > 5)
  {
    return true;
  }
  
  // No Trouble detected
  return false;
} 

bool Alarm_Delay(void)
{ 
  const bool Mode = (0 == digitalRead(MOD_DIN));
  if (Mode != Current_Mode)
  {
    Current_Mode = Mode;

    // Delay - ~ 5 Minutes
    for(U8 i=0; i<35; ++i)
    {
      // Exit ?
      if (!Check_Confirmed()) return false;
      
      // Medium Led Flash
      for(U8 j=0; j<2; ++j)
      {
        digitalWrite(LED_OUT, HIGH);
        delay(10);
        digitalWrite(LED_OUT, LOW);
      
        // Short Nap
        Low_Power(WDT_256_MS);
      }
      
      // Long Nap of 8s 
      Low_Power(WDT_8_SEC);
    }
    
    // force exit
    return false;
  }
  return true;
}

void Alarm_Sequence(void)
{ 
  // Slow Flash
  for(U8 i=0; i<20; ++i)
  {
    if (!Check_Confirmed()) return;

    // Led Flash
    digitalWrite(LED_OUT, HIGH);
    delay(10);
    digitalWrite(LED_OUT, LOW);
    
    // Short Nap
    Low_Power(WDT_1_SEC);

    // Alarm Delay
    if (!Alarm_Delay()) return;
  }

  // Fast Signal
  for(U8 i=0; i<5; ++i)
  {
    if (!Check_Confirmed()) return;

    // Fast Led Flash
    for(U8 j=0; j<4; ++j)
    {
      digitalWrite(LED_OUT, HIGH);
      delay(2);
      digitalWrite(LED_OUT, LOW);
    
      // Short Nap
      Low_Power(WDT_256_MS);
    }
    
    // Alarm Delay
    if (!Alarm_Delay()) return;
  }

  // First Alarm Signal
  for(U8 i=0; i<5; ++i)
  {
    if (!Check_Confirmed()) return;

    // Led + Buz Flash
    digitalWrite(LED_OUT, HIGH);
    digitalWrite(BUZ_OUT, HIGH);
    delay(10);
    digitalWrite(LED_OUT, LOW);
    digitalWrite(BUZ_OUT, LOW);

    // Short Nap
    Low_Power(WDT_1_SEC);
    
    // Alarm Delay
    if (!Alarm_Delay()) return;
  }

  // Second Alarm Signal
  for(U8 i=0; i<20; ++i)
  {
    if (!Check_Confirmed()) return;

    // Led + Buz Flash
    digitalWrite(LED_OUT, HIGH);
    digitalWrite(BUZ_OUT, HIGH);
    delay(10);
    digitalWrite(LED_OUT, LOW);
    delay(90);
    digitalWrite(BUZ_OUT, LOW);

    // Short Nap
    Low_Power(WDT_1_SEC);

    // Alarm Delay
    if (!Alarm_Delay()) return;
  }

  // Thrid Alarm Signal -- no limit
  for(;;)
  {
    if (!Check_Confirmed()) return;
    
    // Led + Buz Flash
    digitalWrite(LED_OUT, HIGH);
    digitalWrite(BUZ_OUT, HIGH);
    delay(10);
    digitalWrite(LED_OUT, LOW);
    delay(490);
    digitalWrite(BUZ_OUT, LOW);

    // Short Nap
    Low_Power(WDT_1_SEC);
    
    // Alarm Delay
    if (!Alarm_Delay()) return;
  }
}
 
// -----------------------------------------------------------//
// ------- Init & Main ---------------------------------------// 
// -----------------------------------------------------------//

void setup() 
{ 
#if ACTIV_TEMP_SENSOR == 1
  // Save ADCSRA value
  Initial_ADCSRA_Value = ADCSRA;
  Initial_ADMUX_Value  = ADMUX;
#endif // ACTIV_TEMP_SENSOR
  
  // Initial Nap
  Low_Power(WDT_16_MS);
  
  // Test Output
  digitalWrite(LED_OUT, HIGH);
  digitalWrite(BUZ_OUT, HIGH);
  delay(100);
  digitalWrite(LED_OUT, LOW);
  digitalWrite(BUZ_OUT, LOW);
  if (0 == digitalRead(MOD_DIN))
  {
    delay(500);
    digitalWrite(LED_OUT, HIGH);
    digitalWrite(BUZ_OUT, HIGH);
    delay(100);
    digitalWrite(LED_OUT, LOW);
    digitalWrite(BUZ_OUT, LOW);
  }
  
  // Save Current Mode
  Current_Mode = (0 == digitalRead(MOD_DIN));
  
  // Test
#if 0
  if (0 == digitalRead(MOD_DIN))
  {
    // Test LDR
    digitalWrite(LDR_OUT, HIGH);
    for(;;)
    {
      signed short value = analogRead(LDR_AIN);
      DebugPrintLed(value);  
      delay(2000);
    }
  }
  else
  {
    // Test TEMP
    for(;;)
    {
      DebugPrintLed(Temp_Value);   
      Low_Power(WDG_TIME_2048MS);
    }
  }
#endif
}

void loop() 
{
  // Alarm Sequence
  Alarm_Sequence(); 

  // Long Nap of 8s 
  Low_Power(WDT_8_SEC);  
}


