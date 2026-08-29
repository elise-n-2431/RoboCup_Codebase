#include "command_router.h"

#include "driving_controller.h"
#include "outputs/smart_servo.h"
#include "inputs/tof_expander.h"

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
            motor_control_stop();
            break;

        case CMD_NONE:
        default:
            break;
    }
}