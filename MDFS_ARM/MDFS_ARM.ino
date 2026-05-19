#include <Servo.h>

const int numServos = 2;
Servo armServos[numServos];
int servoPins[numServos] = {9, 10};

bool isDeposited, isDumped;
int states = 1;

void setup() {
  Serial.begin(9600);

  isDeposited = false;
  isDumped = false;

  for (int i = 0; i < numServos; i++) {
    armServos[i].attach(servoPins[i]);
  }
}

void loop() {

  switch (states) {

    case 1:
      armServos[0].write(0);
      delay(1000);
      armServos[i].write(20);

      isDeposited = true;
      isDumped = false;
      break;

    case 2:
      armServos[0].write(180);
      delay(1000);
      armServos[i].write(65);

      isDeposited = false;
      isDumped = true;
      break;
  }


  if (isDeposited) {
    states = 2;
  }

  if (isDumped) {
    states = 1;
  }

  delay(1000); 
}