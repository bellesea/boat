/*
        Arduino Brushless Motor Control
     by Dejan, https://howtomechatronics.com
*/

#include <PWMServo.h>

PWMServo ESC;   // create servo object to control the ESC
PWMServo ESC2;  // create servo object to control the ESC

int potValue;  // value from the analog pin

void setup() {
  // Attach the ESC on pin 9
  ESC.attach(10, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
  ESC2.attach(9, 1000, 2000);  // (pin, min pulse width, max pulse width in microseconds)
}

void loop() {
  ESC.write(0);   // Send the signal to the ESC
  ESC2.write(0);  // Send the signal to the ESC
}
