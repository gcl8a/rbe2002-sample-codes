#include "serial_comm.h"
#include "PIDcontroller.h"

String serialInput;

bool CheckSerialInput(void)
{
  if(Serial.available())
  {
    char c = Serial.read();
    serialInput += c;
    if(c == '\n') return true;
  }

  return false;
}

void ParseSerialInput(void)
{
  float k = 0;
  switch(serialInput[0])
  {
    case 'P':
      k = serialInput.substring(1).toFloat();
      motorController.SetKp(k);
      break;
    case 'I':
      k = serialInput.substring(1).toFloat();
      motorController.SetKi(k);
      break;
    case 'D':
      k = serialInput.substring(1).toFloat();
      motorController.SetKd(k);
      break;
    case 'L': //target speed
//      float targetLeft = serialInput.substring(1).toFloat();
//      Serial.print("Setting left target to: ");
//      Serial.println(targetLeft);
      break;
    case 'R': //target speed
//      float targetRight = serialInput.substring(1).toFloat();
//      Serial.print("Setting right target to: ");
//      Serial.println(targetRight);
      break;
  }

  serialInput = "";
}
