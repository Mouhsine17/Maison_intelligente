#include <LCD_I2C.h>
#include <HCSR04.h>
#include <AccelStepper.h>

#define MOTOR_INTERFACE_TYPE 4

#define TRIGGER_PIN 9
#define ECHO_PIN 10

#define IN_1 31
#define IN_2 33
#define IN_3 35
#define IN_4 37

#define RED_PIN 2
#define BLUE_PIN 4

#define BUZZER_PIN 5

#define DEG_MIN 10
#define DEG_MAX 170

#define DEG_MINC 57
#define DEG_MAXC 967

#define DIS_COMP1 30
#define DIS_COMP2 60


LCD_I2C lcd(0x27, 16, 2);
HCSR04 hc(TRIGGER_PIN, ECHO_PIN);
AccelStepper myStepper(MOTOR_INTERFACE_TYPE, IN_1, IN_3, IN_2, IN_4);

// Alarme
enum EtatAlarme { NORMAL,
                  ALARME };
EtatAlarme etatAlarme = NORMAL;
unsigned long dernierDeclenchement = 0;

// Temps
unsigned long lastDistanceCheck = 0;
unsigned long lastLcdUpdate = 0;
unsigned long lastSerialPrint = 0;

float distance = 0;
int angle = 0;
int lastAngle = -1;


enum Etat { TROP_PRES,
            TROP_LOIN,
            DANS_ZONE };
Etat etatDistance = TROP_LOIN;

void allumage() {
  lcd.begin();
  lcd.backlight();
  lcd.setCursor(0, 0);
  lcd.print("2349185");
  lcd.setCursor(0, 1);
  lcd.print("Labo 5");
  delay(2000);
  lcd.clear();
}

void setup() {
  Serial.begin(115200);
  myStepper.setMaxSpeed(500);
  myStepper.setAcceleration(100);
  myStepper.setCurrentPosition(0);

  pinMode(BUZZER_PIN, OUTPUT);
  pinMode(RED_PIN, OUTPUT);
  pinMode(BLUE_PIN, OUTPUT);

  allumage();
}

void loop() {
  unsigned long now = millis();


  if (now - lastDistanceCheck >= 50) {
    distance = hc.dist();
    updateEtatDistanceEtAngle();
    updateAlarme();
    lastDistanceCheck = now;
  }


  if (now - lastLcdUpdate >= 100) {
    afficherLCD();
    lastLcdUpdate = now;
  }


  if (now - lastSerialPrint >= 100) {
    Serial.print("etd:");
    Serial.print("2349185");
    Serial.print(",dist:");
    Serial.print((int)distance);
    Serial.print(",deg:");
    Serial.println(angle);
    lastSerialPrint = now;
  }

  // Gestion moteur
  if (etatDistance == DANS_ZONE) {
    if (angle != lastAngle) {
      int steps = map(angle, DEG_MIN, DEG_MAX, DEG_MINC, DEG_MAXC);
      myStepper.enableOutputs();
      myStepper.moveTo(steps);
      lastAngle = angle;
    }
    myStepper.run();
  } else {
    myStepper.disableOutputs();
  }
}

void updateEtatDistanceEtAngle() {
  if (distance < DIS_COMP1) {
    etatDistance = TROP_PRES;
    angle = 10;
  } else if (distance > DIS_COMP2) {
    etatDistance = TROP_LOIN;
    angle = 170;
  } else {
    etatDistance = DANS_ZONE;
    angle = map(distance, DIS_COMP1, DIS_COMP2, DEG_MIN, DEG_MAX);
  }
}

void afficherLCD() {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Dist:");
  lcd.print((int)distance);
  lcd.print("cm");
  lcd.setCursor(0, 1);
  if (etatDistance == TROP_PRES) {
    lcd.print("Trop pres");
  } else if (etatDistance == TROP_LOIN) {
    lcd.print("Trop loin");
  } else {
    lcd.print("Angle:");
    lcd.print(angle);
    lcd.print("deg");  // Degré
  }
}

void updateAlarme() {
  if (distance <= 15) {
    etatAlarme = ALARME;
    dernierDeclenchement = millis();
  } else if (millis() - dernierDeclenchement >= 3000) {
    etatAlarme = NORMAL;
  }

  if (etatAlarme == ALARME) {
    tone(BUZZER_PIN, 1000);  // Sonne à 1kHz

    if (millis() % 500< 250) {
      digitalWrite(RED_PIN, HIGH);
      digitalWrite(BLUE_PIN, LOW);
    } else {
      digitalWrite(RED_PIN, LOW);
      digitalWrite(BLUE_PIN, HIGH);
    }

  } else {
    noTone(BUZZER_PIN);  
    digitalWrite(RED_PIN, LOW);
    digitalWrite(BLUE_PIN, LOW);
  }
}
