/* Arduino metal detector; (for Arduino Uno / 5v Pro Mini)
*   serial port used for status output and receiving control commands
*  D13 alert pin (alert low)
*/ 
#define ALERT_PIN 13
#define SerialRate 115200
// Number of cycles from external counter needed to generate a signal event
#define CYCLES_PER_SIGNAL 5000
// Frequency delta threshold for ALERT to trigger
#define ALERT_THRESHOLD 49
// Common Pin definitions
#define SENSITIVITY_POT_APIN A0
// #define RESET_BTN_PIN 12		not implemented yet!

unsigned long lastSignalTime = 0;
unsigned long signalTimeDelta = 0;
boolean firstSignal = true;
unsigned long storedTimeDelta = 0;
int lastTriggerValue=0;      // stored trigger value to detect change

// This signal is called whenever OCR1A reaches 0
// (Note: OCR1A is decremented on every external clock cycle)
SIGNAL(TIMER1_COMPA_vect)
{
  unsigned long currentTime = micros();
  signalTimeDelta =  currentTime - lastSignalTime;
  lastSignalTime = currentTime;

  if (firstSignal)
  {
    firstSignal = false;
  }
  else if (storedTimeDelta == 0)
  {
    storedTimeDelta = signalTimeDelta;
  }

  // Reset OCR1A
  OCR1A += CYCLES_PER_SIGNAL;
}

void setup()
{
   Serial.begin(SerialRate);	// Setup serial interface for test data outputs

 // INITIALISE METAL DETECTOR TIMER 
  // Set WGM(Waveform Generation Mode) to 0 (Normal)
  TCCR1A = 0b00000000;
    // Set CSS(Clock Speed Selection) to 0b111 (External clock source on T0 pin
  // (ie, pin 5 on UNO). Clock on rising edge.)
  TCCR1B = 0b00000111;
  // Enable timer compare interrupt A (ie, SIGNAL(TIMER1_COMPA_VECT))
  TIMSK1 |= (1 << OCIE1A);
  // Set OCR1A (timer A counter) to 1 to trigger interrupt on next cycle
  OCR1A = 1;

  pinMode(ALERT_PIN, OUTPUT);

  // DEBUG ONLY - show adj pin value
  Serial.print("adj=");
  Serial.println(analogRead(SENSITIVITY_POT_APIN));
}

void loop()
{

    float sensitivity = mapFloat(analogRead(SENSITIVITY_POT_APIN), 0, 1023, 0.5, 10.0);
    int storedTimeDeltaDifference = (storedTimeDelta - signalTimeDelta) * sensitivity;
    if(storedTimeDeltaDifference!=lastTriggerValue) 
      { Serial.print("val=");
        Serial.println(storedTimeDeltaDifference);
      }  
    if (storedTimeDeltaDifference > ALERT_THRESHOLD)
    {
      digitalWrite(ALERT_PIN, LOW);
      Serial.print("alert triggered; val=");
      Serial.println(storedTimeDeltaDifference);
    }
    else
    {
      digitalWrite(ALERT_PIN, HIGH);
    }
// reset not implemented yet
//  if (digitalRead(RESET_BTN_PIN) == LOW)
// {
//  storedTimeDelta = 0;
// }
  delay(500);
    
 }



float mapFloat(int input, int inMin, int inMax, float outMin, float outMax)
{
  float scale = (float)(input - inMin) / (inMax - inMin);
  return ((outMax - outMin) * scale) + outMin;
}
