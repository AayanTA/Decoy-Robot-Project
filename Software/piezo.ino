// Guide to tones
// https://docs.arduino.cc/built-in-examples/digital/toneMelody/

// Try to recreate this sound of a rifle
// https://www.youtube.com/watch?v=KJIqrmcwHqU

const int buzzerPin = 9; //buzzer to arduino pin 9


void setup(){
   
  pinMode(buzzerPin, OUTPUT); // Set buzzer - pin 9 as an output

}

void   loop(){
 
  tone(buzzerPin, 1000); // Send 1KHz sound signal...
  delay(1000);         // ...for 1 sec
  noTone(buzzerPin);     // Stop sound...
  delay(1000);         // ...for 1sec
  
}
