#include <Servo.h>

// global variable
int angle = 0;
Servo ackermann_chassis;
String command;

void setup() {
  ackermann_chassis.attach(10);
  Serial.begin(115200);
}

void loop() {
  while (Serial.available() > 0) {
    char temp = char(Serial.read());
    command += temp;
  }

  if (command.length() > 0) {
    int begin = command.indexOf('<');
    int end = command.indexOf('>') + 1;
    if (begin != -1 && end != -1 && begin < end) {
      sscanf(command.substring(begin, end).c_str(), "<%d>", &angle);
      ackermann_chassis.write(angle);
    }
  }
  command = "";
  delay(15);
}
