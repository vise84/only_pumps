#include <Arduino.h>
#include <RTClib.h>

#define BUTTON1        11
#define IN1            8
#define IN2            7
#define LED1           12

#define BUTTON2        9
#define IN3            6
#define IN4            5
#define LED2           10

RTC_DS3231 rtc;

enum PumpID {
    PUMP1,
    PUMP2
};

// Settings
// scheduling
const unsigned long onByButtonTimer = 3600000; // 60 minutes in milliseconds
const unsigned long offByButtonTimer = 7200000; // 120 minutes in milliseconds

const int P1turnOnHH = 1;
const int P1turnOffHH = 5;
const bool P1turnOnMonths[12] = {false, false, false, true, true, true, true, true, true, false, false, false};
const int P1skipDaysPerMonth[12]= {0, 0, 0, 5, 1, 0, 0, 0, 1, 0, 0, 0};

const int P2turnOnHH = 1;
const int P2turnOffHH = 5;
const bool P2turnOnMonths[12] = {false, false, false, true, true, true, true, true, true, false, false, false};
const int P2skipDaysPerMonth[12]= {0, 0, 0, 5, 2, 1, 1, 1, 2, 0, 0, 0};

// Initialize state
// pumps
bool P1isOn = false;
unsigned long P1buttonPressRecentUntil = 0;
unsigned long P1turnedOffAt = 0;

bool P2isOn = false;
unsigned long P2buttonPressRecentUntil = 0;
unsigned long P2turnedOffAt = 0;

DateTime customTime;


void setup() {
    Serial.begin(9600);
    
    pinMode(IN1, OUTPUT);
    pinMode(IN2, OUTPUT);
    pinMode(IN3, OUTPUT);
    pinMode(IN4, OUTPUT);
    pinMode(LED1, OUTPUT);
    pinMode(LED2, OUTPUT);
    pinMode (BUTTON1, INPUT_PULLUP);
    pinMode (BUTTON2, INPUT_PULLUP);
   
    digitalWrite (IN1,LOW);
    digitalWrite (IN2,LOW);
    digitalWrite (IN3,LOW);
    digitalWrite (IN4,LOW);

}

DateTime now() {
    // Set date and time (format: Year, Months, Day, Hours, Minute, Seconds)
    rtc.adjust(DateTime(2024, 6, 11, 2, 0, 1));
    return rtc.now();
}


bool IsPumpOn(PumpID pump) {
   if (pump == PUMP1) {
       return P1isOn;
   } else if (pump == PUMP2) {
       return P2isOn;
   }
   return false;
}

bool IsItTimeToBeOn(PumpID pump) {
  int turnOnHH, turnOffHH;
  const int* skipDaysPerMonth;
  const bool* turnOnMonths;
  unsigned long turnedOffAt;

    if (pump == PUMP1) {
        turnOnHH = P1turnOnHH;
        turnOffHH = P1turnOffHH;
    turnOnMonths = P1turnOnMonths;
        skipDaysPerMonth = P1skipDaysPerMonth;
       turnedOffAt = P1turnedOffAt;
    } else if (pump == PUMP2) {
        turnOnHH = P2turnOnHH;
        turnOffHH = P2turnOffHH;
        turnOnMonths = P2turnOnMonths;
        skipDaysPerMonth = P2skipDaysPerMonth;
        turnedOffAt = P2turnedOffAt;
    } else {
        return false;
    }

  int hh = now().hour();
  int mm = now().month();
  if (turnOnMonths[mm-1] == true ){
    if (hh >= turnOnHH && hh < turnOffHH) {
      unsigned long skippedDays = (now().unixtime() - turnedOffAt) / 86400;
      return skippedDays >= skipDaysPerMonth[mm-1];
    }
  }
    
  return false;
}

bool IsButtonPressed(PumpID pump) {
      if (pump == PUMP1) {
       if (digitalRead (BUTTON1) == 0) {
        Serial.println("BUTTON1 pressed");
      return true;
       } 
    return false;
      }
    if (pump == PUMP2) {
       if (digitalRead (BUTTON2) == 0) {
        Serial.println("BUTTON2 pressed");
      return true;
        } 
    return false;
      }
}

bool WasButtonPressedRecently(PumpID pump) {
    unsigned long recencyThreshold = (pump == PUMP1) ? P1buttonPressRecentUntil : P2buttonPressRecentUntil;
    return now().unixtime() <= recencyThreshold;
}

void TurnOnPump(PumpID pump, bool triggeredByButton) {
    if (pump == PUMP1) {
        digitalWrite(IN1, HIGH);  // Imposta il pin IN1 HIGH 
        digitalWrite(IN2, LOW);   // Imposta il pin IN2 LOW 
        delay(2000);
        digitalWrite(IN1, LOW);   // Imposta il pin IN1 LOW 
        digitalWrite(IN2, LOW);   // Imposta il pin IN2 LOW 
        P1isOn = true;
        Serial.println("PUMP1 ON");
    } else if (pump == PUMP2) {
        // TODO: Add code to turn on P2
        digitalWrite(IN3, HIGH);  // Imposta il pin IN1 HIGH 
        digitalWrite(IN4, LOW);   // Imposta il pin IN2 LOW 
      delay(2000);
        digitalWrite(IN3, LOW);   // Imposta il pin IN1 LOW 
        digitalWrite(IN4, LOW);   // Imposta il pin IN2 LOW 
       P2isOn = true;
       Serial.println("PUMP2 ON");
   }

    if (triggeredByButton) {
        if (pump == PUMP1) {
            P1buttonPressRecentUntil = now().unixtime() + onByButtonTimer;
        } else if (pump == PUMP2) {
            P2buttonPressRecentUntil = now().unixtime() + onByButtonTimer;
        }
    }
}

void TurnOffPump(PumpID pump, bool triggeredByButton) {
    if (pump == PUMP1) {
        // TODO: Add code to turn off P1
        digitalWrite(IN1, LOW);   // Imposta il pin IN1 LOW 
       digitalWrite(IN2, HIGH);   // Imposta il pin IN2 HIGH 
       delay(2000);
       digitalWrite(IN1, LOW);   // Imposta il pin IN1 LOW 
        digitalWrite(IN2, LOW);   // Imposta il pin IN2 LOW 
        P1isOn = false;
        P1turnedOffAt = now().unixtime();
        Serial.println("PUMP1 OFF");
    } else if (pump == PUMP2) {
        // TODO: Add code to turn off P2
        digitalWrite(IN3, LOW);   // Imposta il pin IN1 LOW 
        digitalWrite(IN4, HIGH);   // Imposta il pin IN2 HIGH 
        delay(2000);
        digitalWrite(IN3, LOW);   // Imposta il pin IN1 LOW 
        digitalWrite(IN4, LOW);   // Imposta il pin IN2 LOW 
        P2isOn = false;
        Serial.println("PUMP2 OFF");
       P2turnedOffAt = now().unixtime();
    }

    if (triggeredByButton) {
        if (pump == PUMP1) {
            P1buttonPressRecentUntil = now().unixtime() + offByButtonTimer;
        } else if (pump == PUMP2) {
            P2buttonPressRecentUntil = now().unixtime() + offByButtonTimer;
        }
    }
}

void loop() {
    Serial.println("START");
    Serial.println(PumpID());
    PumpID pumps[] = {PUMP1, PUMP2};
    Serial.println(PumpID());
    for (PumpID pump : pumps) {
        bool pumpIsOn = IsPumpOn(pump);
        bool itsTimeToBeOn = IsItTimeToBeOn(pump);
        bool buttonIsPressed = IsButtonPressed(pump);
       bool buttonPressedRecently = WasButtonPressedRecently(pump);

     if (!pumpIsOn) {
            if (buttonIsPressed) {
                TurnOnPump(pump, true);
            } else if (itsTimeToBeOn && !buttonPressedRecently) {
               TurnOnPump(pump, false);
           }
        } else {
            if (buttonIsPressed) {
                TurnOffPump(pump, true);
            } else if (!itsTimeToBeOn && !buttonPressedRecently) {
                TurnOffPump(pump, false);
           }
        }
    }      
}
