// This example reads the raw values from the LSM6DS33
// accelerometer and gyro and prints those raw values to the
// serial monitor.

#include <Romi32U4.h>

Romi32U4ButtonA buttonA;
Romi32U4ButtonB buttonB;
Romi32U4ButtonC buttonC;
Romi32U4Motors motors;

LSM6 imu;

void setup()
{
  Wire.begin();

  if (!imu.init())
  {
    // Failed to detect the LSM6.
    ledRed(1);
    while(1)
    {
      Serial.println(F("Failed to detect the LSM6."));
      delay(100);
    }
  }

  imu.enableDefault();

  // Set the gyro full scale and data rate
  imu.setGyroDataOutputRate(LSM6::ODR13);

  // Set the accelerometer full scale and data rate
  imu.setAccDataOutputRate(LSM6::ODR13);
}

bool showAcc = true;
bool showGyro = false;

void loop()
{
  if(buttonA.isPressed()) motors.setEfforts(200, 200);
  else motors.setEfforts(0,0);

  if(buttonB.getSingleDebouncedPress()) showAcc = !showAcc;
  if(buttonC.getSingleDebouncedPress()) showGyro = !showGyro;

  if(imu.getStatus() & 0x01)
  {
    imu.read();

    Serial.print(millis());
    Serial.print('\t');

    if(showAcc)
    {
      Serial.print(imu.a.x);
      Serial.print(' ');
      Serial.print(imu.a.y);
      Serial.print(' ');
      Serial.print(imu.a.z);
      Serial.print(' ');
    }

    if(showGyro)
    {
      Serial.print(imu.g.x);
      Serial.print(' ');
      Serial.print(imu.g.y);
      Serial.print(' ');
      Serial.print(imu.g.z);
      Serial.print(' ');
    }

    Serial.print('\n');
  }
}
