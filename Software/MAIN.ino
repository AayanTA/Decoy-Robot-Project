// Main code

#include "pitches.h"
#include <Servo.h>

// Servo objects
Servo leftServo;
Servo rightServo;

// constants won't change. They're used here to set pin numbers:
const int buttonPin = 2;  // the number of the pushbutton pin
const int ledPin = 9;    // the number of the LED pin
const int dialPin = A0;  // Analog input pin that the potentiometer is attached to
// Ultrasonic sensor connections
const int trigPin = 12;  // Trigger pin for the ultrasonic sensor
const int echoPin = 13;  // Echo pin for the ultrasonic sensor
// Distance threshold for stopping (in centimeters)
const int distanceThreshold = 15;  // Stop if obstacle is closer than 15 cm

// variables will change
int dialValue = 0;        // value read from the pot
int timerValue = 0;        // value output to the PWM (analog out)
bool ledState = LOW;  // ledState used to set the LED
int buttonState;            // the current reading from the input pin
int lastButtonState = LOW;  // the previous reading from the input pin
int timerDelay = 10000; // the mapped reading from the pot for delaying actions from the button press time
bool active = false; // variable to check if the robot should be moving

// the following variables are unsigned longs because the time, measured in
// milliseconds, will quickly become a bigger number than can be stored in an int.
unsigned long lastDebounceTime = 0;  // the last time the output pin was toggled
unsigned long debounceDelay = 50;    // the debounce time; increase if the output flickers

// Generally, you should use "unsigned long" for variables that hold time
// The value will quickly become too large for an int to store
unsigned long previousMillis = 0;  // will store last time LED was updated
unsigned long buttonMillis = 0;
unsigned long soundMillis = 0;


// constants won't change:
const long interval = 1000;  // interval at which to blink (milliseconds)


unsigned long pauseBetweenNotes;
int thisNote;

// notes in the melody:
int melody[] = {

  NOTE_B3, NOTE_CS3, NOTE_D3, NOTE_E3, NOTE_F3, NOTE_G3
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {

  4, 4, 8, 4, 4, 8
};

void setup() {
  // initialize serial communications at 9600 bps:
  Serial.begin(9600);
  
  // Attach servos to their pins
  leftServo.attach(13);   // Attach left servo to Pin 13
  rightServo.attach(12); // Attach right servo to Pin 12

  // Set ultrasonic sensor pins
  pinMode(trigPin, OUTPUT);  // Set trigger pin as output
  pinMode(echoPin, INPUT);   // Set echo pin as input
  
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  
  // set initial LED state
  digitalWrite(ledPin, ledState);
}

long measureDistance() {
  // Send a 10us pulse to trigger the sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Measure the pulse duration on the echo pin
  long duration = pulseIn(echoPin, HIGH);

  // Convert duration to distance (in cm)
  long distance = duration * 0.034 / 2;

  return distance;
}

// Function to move the robot forward
void moveForward() {
  leftServo.writeMicroseconds(1300);  // Full speed forward for left servo (counterclockwise)
  rightServo.writeMicroseconds(1700);   // Full speed forward for right servo (clockwise)
}
// Function to move the robot backward
void moveBackward() {
  leftServo.writeMicroseconds(1700);    // Full speed backward for left servo
  rightServo.writeMicroseconds(1300); // Full speed backward for right servo
}
// Function to turn the robot left
void turnLeft() {
  leftServo.writeMicroseconds(1500);    // Stop left servo
  rightServo.writeMicroseconds(1700);   // Right servo moves forward
}
// Function to turn the robot right
void turnRight() {
  leftServo.writeMicroseconds(1300);  // Left servo moves forward
  rightServo.writeMicroseconds(1500); // Stop right servo
}
// Function to stop the robot
void stopRobot() {
  servoLeft.writeMicroseconds(1500); // 1.5 ms stay-still signal
  rightServo.writeMicroseconds(1500);  // Stop right servo
}

void playSound() {

  // iterate over the notes of the melody:

  for (int thisNote = 0; thisNote < 6; thisNote++) {
    // iterate over the notes of the melody:
    if (thisNote < 6 && millis() - previousMillis >= pauseBetweenNotes) {
      previousMillis = millis();

      // to calculate the note duration, take one second divided by the note type.
      //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
      int noteDuration = 1000 / noteDurations[thisNote];
      tone(8, melody[thisNote], noteDuration);

      // to distinguish the notes, set a minimum time between them.
      // the note's duration + 30% seems to work well:
      pauseBetweenNotes = noteDuration * 1.30;
      
      //thisNote++;
    }
  }
}

void readButton() {
  // read the state of the switch into a local variable:
  int reading = digitalRead(buttonPin);

  // check to see if you just pressed the button
  // (i.e. the input went from LOW to HIGH), and you've waited long enough
  // since the last press to ignore any noise:

  // If the switch changed, due to noise or pressing:
  if (reading != lastButtonState) {
    // reset the debouncing timer
    lastDebounceTime = millis();
  }

  if ((millis() - lastDebounceTime) > debounceDelay) {
    // whatever the reading is at, it's been there for longer than the debounce
    // delay, so take it as the actual current state:

    // if the button state has changed:
    if (reading != buttonState) {
      buttonState = reading;

      // only toggle the LED if the new button state is HIGH
      if (buttonState == HIGH) {
        buttonMillis = millis();
        active = !active;
        timerDelay = map(dialValue, 0, 1023, 0, 30000);
        Serial.println(timerDelay);
        Serial.print(" Timer Value ");
        //Serial.println(active);
        //Serial.print("  Hello  ");
      }
    }
  }

  // save the reading. Next time through the loop, it'll be the lastButtonState:
  lastButtonState = reading;
}

void movementLogic() {
  moveForward();
}

void loop() {
  //Serial.println("loopy");
  unsigned long currentMillis = millis();
  dialValue = analogRead(dialPin);
  readButton();
  if (active == true) {
    //Serial.println("active!!");
    digitalWrite(ledPin, HIGH);
    if (currentMillis - buttonMillis >= timerDelay) {
      //Serial.println("hihi");
      playSound();
      if (currentMillis - previousMillis >= interval) {
        // save the last time you blinked the LED
        //Serial.println("millissssssssssssssssssssssss");
        previousMillis = currentMillis;
    
        // if the LED is off turn it on and vice-versa:
        if (ledState == HIGH) {
          ledState = LOW;
        } else {
          ledState = HIGH;
        }
        // set the LED with the ledState of the variable:
        Serial.println(ledState);
        digitalWrite(ledPin, ledState);
      }
    }
  } else {
    digitalWrite(ledPin, LOW);
  }
}
