#include "command_router.h"

#include "navigator.h"
#include "outputs/smart_servo.h"


void command_router_exe(RobotCommand command)
{
    switch (command)
    {
        case OPEN_GATE:
            smartservo_gate_open();
            break;

        case CLOSE_GATE:
            smartservo_gate_close();
            break;

        case CMD_STOP:
            navigator_stop();
            break;
            
        default:
            break;
    }
}