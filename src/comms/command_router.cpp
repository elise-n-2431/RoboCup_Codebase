#include "command_router.h"

#include "navigator.h"
#include "outputs/pickup_servo.h"


void command_router_exe(RobotCommand command)
{
    switch (command)
    {
        case OPEN_GATE:
            gateOpen();
            break;

        case CLOSE_GATE:
            gateClose();
            break;

        case CMD_STOP:
            navigator_stop();
            break;
            
        default:
            break;
    }
}