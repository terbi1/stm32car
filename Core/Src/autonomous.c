#include "autonomous.h"

static inline uint8_t DegreeToCount(uint8_t degree, uint8_t countsPerRev) {
    return degree * countsPerRev / 180; 
}


void AutonomousController_Init(AutonomousController *autoCtl)
{
    autoCtl->looKingDirection = FRONT;
    // autoCtl->servoDirection = FRONT;
    autoCtl->goingDirection = FRONT;
    autoCtl->carState = MOVE;
    autoCtl->scanDone = 0;
    autoCtl->turning_ticks = 0;
    autoCtl->frontDistance = 400;
    autoCtl->leftDistance = 0;
    autoCtl->rightDistance = 0;
    autoCtl->isBlocked = 0;
}

void AutonomousController_NextState()
{
}
void AutonomousController_Update(AutonomousController *autoCtl)
{
    switch (autoCtl->carState)
    {
    case SCAN:
        if (!autoCtl->scanDone)
            break;

        if (autoCtl->leftDistance > autoCtl->rightDistance)
        {
            autoCtl->goingDirection = LEFT;
            autoCtl->carState = MOVE;
        }
        else if (autoCtl->leftDistance <= autoCtl->rightDistance)
        {
            autoCtl->goingDirection = RIGHT;
            autoCtl->carState = MOVE;
        }
        autoCtl->scanDone = 0;
        autoCtl->turning_ticks = 0;
        break;
    case MOVE:
        switch (autoCtl->goingDirection)
        {
        case FRONT:
            if (autoCtl->frontDistance <= AUTO_CRITICAL_DISTANCE && !autoCtl->isBlocked)
            {
                autoCtl->isBlocked = 1;
                autoCtl->goingDirection = IDLE;
            }
            break;
        case BACK:
            if (autoCtl->turning_ticks >= COUNTS_PER_REV)
            {
                autoCtl->carState = SCAN;
                // autoCtl->turning_ticks = 0;
                autoCtl->leftDistance = 0;
                autoCtl->rightDistance = 0;
            }
            break;
        case IDLE:
            if (autoCtl->isBlocked)
                autoCtl->goingDirection = BACK;
            else
                autoCtl->goingDirection = FRONT;
            break;
        case LEFT:
            if (autoCtl->turning_ticks >= DegreeToCount(45, COUNTS_PER_REV))
            {
                autoCtl->leftDistance = 0;
                autoCtl->rightDistance = 0;
                autoCtl->looKingDirection = FRONT;
                autoCtl->goingDirection = IDLE;
                autoCtl->isBlocked = 0;
                autoCtl->turning_ticks = 0;
            }
            break;
        case RIGHT:
            if (autoCtl->turning_ticks >= DegreeToCount(45, COUNTS_PER_REV))
            {
                autoCtl->leftDistance = 0;
                autoCtl->rightDistance = 0;
                autoCtl->looKingDirection = FRONT;
                autoCtl->goingDirection = IDLE;
                autoCtl->isBlocked = 0;
                autoCtl->turning_ticks = 0;
            }
            break;
        default:
            break;
        }
    default:
        break;
    }
}