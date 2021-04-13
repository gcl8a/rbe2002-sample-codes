#include "chassis.h"

#include <math.h>

volatile int16_t countsLeft = 0;
volatile int16_t countsRight = 0;

Romi32U4Encoders encoders;
Romi32U4Motors motors;

Chassis::Chassis(void) 
    //we'll use the member initializer capabilities of C++ to set up the controllers
    : leftMotorController(5, 1, 0, 500), rightMotorController(5, 1, 0, 500)  
    {
        SetTargetSpeeds(0, 0);
    }

void Chassis::Init(void)
{  
    noInterrupts(); //disable interupts while we mess with the Timer4 registers
  
    //sets up timer 4
    TCCR4A = 0x00; //disable some functionality -- no need to worry about this
    TCCR4B = 0x0B; //sets the prescaler -- look in the handout for values
    TCCR4C = 0x04; //toggles pin 6 at the timer frequency
    TCCR4D = 0x00; //normal mode

    /*
    * EDIT THE LINE BELOW WITH YOUR VALUE FOR TOP
    */

    OCR4C = 249;   //TOP goes in OCR4C 
    timestepMS = 16; //should correspond to your choice for TOP

    TIMSK4 = 0x04; //enable overflow interrupt

    interrupts(); //re-enable interrupts

    //pinMode(6, OUTPUT); //COMMENT THIS OUT TO SHUT UP THE PIEZO!!!
}

void Chassis::UpdatePose(void)
{
    //TO BE COMPLETED BY THE STUDENT

    //conversion from ticks/interval to cm/sec
    float conversionToCMPerSec = 1; //YOU'LL NEED TO CALCULATE THIS VALUE
    
    float spLeft = speedLeft * conversionToCMPerSec;
    float spRight = speedRight * conversionToCMPerSec;

    //average speed
    float u_0 = 0; //YOU'LL NEED TO ADD THIS EXPRESSION

    //omega
    float omega = 0; //YOU'LL NEED TO ADD THIS EXPRESSION

    //simple first-order method -- not sufficient for class
    float dt = timestepMS / 1000.; //SET timestepMS IN THE CONSTRUCTOR

    //YOU'LL NEED TO CALCULATE THESE
    x += 0;
    y += 0;
    theta += 0;

    Serial.print(millis());
    Serial.print('\t');
    Serial.print(x);
    Serial.print('\t');
    Serial.print(y);
    Serial.print('\t');
    Serial.print(theta); 
    Serial.print('\n');
}

void Chassis::UpdateSpeeds(void)
{
    /*
     * Do PID stuffs here. Note that we turn off interrupts while we read countsLeft/Right
     * so that it won't get accidentally updated (in the ISR) while we're reading it.
     */
    noInterrupts();

    speedLeft = countsLeft - prevEncLeft;
    prevEncLeft = countsLeft;
    speedRight = countsRight - prevEncRight;
    prevEncRight = countsRight;

    interrupts();

    int16_t errorLeft = targetSpeedLeft - speedLeft; //calculate the error
    float effortLeft = leftMotorController.ComputeEffort(errorLeft); //calculate effort from error
    
    motors.setLeftEffort(effortLeft); 

    int16_t errorRight = targetSpeedRight - speedRight; //calculate the error
    float effortRight = rightMotorController.ComputeEffort(errorRight); //calculate effort from error
    
    motors.setRightEffort(effortRight); 
}

/*
 * ISR for timing. On overflow, it takes a 'snapshot' of the encoder counts and raises a flag to let
 * the main program it is time to execute the PID calculations.
 */
ISR(TIMER4_OVF_vect)
{
  //Capture a "snapshot" of the encoder counts for later processing
  countsLeft = encoders.getCountsLeft();
  countsRight = encoders.getCountsRight();

  PIDController::readyToPID = 1;
}
