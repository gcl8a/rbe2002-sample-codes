#include "standoff.h"
#include "ir_codes.h"

void StandoffController::processDistanceReading(float distance)
{
    float error = distance - targetDistance;
    float effort = piStandoffer.computeEffort(error);

    leftEffort = effort;
    rightEffort = effort;

    // Serial.print(targetDistance);
    // Serial.print('\t');
    // Serial.print(distance);
    // Serial.print('\t');
    // Serial.print(error);
    // Serial.print('\t');
    // Serial.print(effort);
    // Serial.print('\t');
    // Serial.print(leftEffort);
    // Serial.print('\t');
    // Serial.print(rightEffort);
    // Serial.print('\t');
}

void StandoffController::handleKeyPress(int16_t key)
{
    switch(key)
    {
        case CHplus:
            targetDistance += 10;
            break;

        case CHminus:
            targetDistance -= 10;
            break;

        default:
            if(key >= NUM_0 && key <= NUM_9)
            {
                targetDistance = 10 * (key - NUM_0);
            }
            break;
    }
}