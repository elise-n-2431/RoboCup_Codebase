#include "tasks.h"
#include "state_machine.h"
#include "logic_engine.h"
#include "comms/serial.h"
#include "comms/command_router.h"
#include "outputs/pickup_servo.h"
#include "outputs/emag.h"
#include "outputs/smart_servo.h"
#include "inputs/tof_expander.h"
#include "inputs/proximity.h"
#include "inputs/limit_switch.h"
#include "inputs/xy_sensor.h"
#include "inputs/colour_sensor.h"
#include "inputs/imu.h"
#include "inputs/ultrasound.h"
#include "navigator.h"
#include "pose.h"
#include "map.h"

//----------------------------------------------------------------------
// Periods (ms). Tune these — see notes at the bottom of the file.
//----------------------------------------------------------------------
#define XY_PERIOD               5
#define IMU_PERIOD              5
#define POSE_PERIOD             5
#define TOF_PERIOD             10
#define MAP_PERIOD             60
#define MAP_SEND_PERIOD        300
#define ULTRASOUND_PERIOD      50
#define LIMIT_PERIOD           10
#define LOGIC_PERIOD           10
#define STATE_PERIOD           10
#define PICKUP_EXE_PERIOD      10
#define EMAG_PERIOD            20
#define PROXIMITY_PERIOD       10
#define PICKUP_UPDATE_PERIOD   20
#define COLOUR_PERIOD          50
#define SMARTSERVO_PERIOD      20
#define COMMS_PERIOD            5
#define NAVIGATOR_PERIOD       10

// serial_exe() returns a value, so it needs a void wrapper
static void comms_cb(void)
{
    RobotCommand command = serial_exe();
    command_router_exe(command);
}

Scheduler taskManager;

// Task(interval_ms, iterations, callback)
// TASK_FOREVER == -1
static Task tXy            (XY_PERIOD,             TASK_FOREVER, &xy_exe);
static Task tImu           (IMU_PERIOD,            TASK_FOREVER, &imu_update);
static Task tTof           (TOF_PERIOD,            TASK_FOREVER, &tof_update);
static Task tPose          (POSE_PERIOD,           TASK_FOREVER, &pose_update);
static Task tMap           (MAP_PERIOD,            TASK_FOREVER, &map_update);
static Task tMapSend       (MAP_SEND_PERIOD,       TASK_FOREVER, &send_map_data);
static Task tUltrasound    (ULTRASOUND_PERIOD,     TASK_FOREVER, &ultrasound_exe);
static Task tLimit         (LIMIT_PERIOD,          TASK_FOREVER, &limit_switch_exe);
static Task tLogic         (LOGIC_PERIOD,          TASK_FOREVER, &logic_exe);
static Task tState         (STATE_PERIOD,          TASK_FOREVER, &updateStateMachine);
static Task tPickupExe     (PICKUP_EXE_PERIOD,     TASK_FOREVER, &pickup_servo_exe);
static Task tEmag          (EMAG_PERIOD,           TASK_FOREVER, &emag_exe);
static Task tProximity     (PROXIMITY_PERIOD,      TASK_FOREVER, &proximity_exe);
static Task tPickupUpdate  (PICKUP_UPDATE_PERIOD,  TASK_FOREVER, &pickup_servo_update);
static Task tColour        (COLOUR_PERIOD,         TASK_FOREVER, &colour_sensor_update);
static Task tSmartservo    (SMARTSERVO_PERIOD,     TASK_FOREVER, &smartservo_update);
static Task tComms         (COMMS_PERIOD,          TASK_FOREVER, &comms_cb);
static Task tNavigator     (NAVIGATOR_PERIOD,      TASK_FOREVER, &navigator_exe);

// Name lookup for runtime tuning
struct TaskEntry { const char* name; Task* task; };

static TaskEntry TASK_TABLE[] = {
    { "xy",            &tXy           },
    { "imu",           &tImu          },
    { "tof",           &tTof          },
    { "pose",          &tPose         },
    { "map",           &tMap          },
    { "map_send",      &tMapSend      },
    { "ultrasound",    &tUltrasound   },
    { "limit",         &tLimit        },
    { "logic",         &tLogic        },
    { "state",         &tState        },
    { "pickup_exe",    &tPickupExe    },
    { "emag",          &tEmag         },
    { "proximity",     &tProximity    },
    { "pickup_update", &tPickupUpdate },
    { "colour",        &tColour       },
    { "smartservo",    &tSmartservo   },
    { "comms",         &tComms        },
    { "navigator",     &tNavigator    },
};

static const uint8_t TASK_COUNT = sizeof(TASK_TABLE) / sizeof(TASK_TABLE[0]);

void tasks_init(void)
{
    taskManager.init();

    // Add order matters: within a single execute() pass, tasks run in the
    // order they were added. Keep sensor -> fusion -> logic -> output.
    for (uint8_t i = 0; i < TASK_COUNT; i++)
        taskManager.addTask(*TASK_TABLE[i].task);

    // Stagger the start so slow tasks don't all pile onto the same tick.
    // enableDelayed(n) enables the task but holds the first run for n ms.
    for (uint8_t i = 0; i < TASK_COUNT; i++)
        TASK_TABLE[i].task->enableDelayed(i);
}

void tasks_exe(void)
{
    taskManager.execute();
}

static Task* find(const char* name)
{
    for (uint8_t i = 0; i < TASK_COUNT; i++)
        if (strcmp(TASK_TABLE[i].name, name) == 0) return TASK_TABLE[i].task;
    return nullptr;
}

bool tasks_set_period_ms(const char* name, unsigned long ms)
{
    Task* t = find(name);
    if (!t) return false;
    t->setInterval(ms);   // note: resets the countdown from now
    return true;
}

bool tasks_enable(const char* name, bool on)
{
    Task* t = find(name);
    if (!t) return false;
    on ? t->enable() : t->disable();
    return true;
}

void tasks_report(Stream& out)
{
#ifdef _TASK_TIMECRITICAL
    out.println(F("task            interval  overrun  en"));
    for (uint8_t i = 0; i < TASK_COUNT; i++) {
        Task* t = TASK_TABLE[i].task;
        out.print(TASK_TABLE[i].name);
        for (int p = strlen(TASK_TABLE[i].name); p < 16; p++) out.print(' ');
        out.print(t->getInterval());   out.print('\t');
        out.print(t->getOverrun());    out.print('\t');   // negative == late
        out.println(t->isEnabled() ? '1' : '0');
    }
#else
    out.println(F("rebuild with -D_TASK_TIMECRITICAL"));
#endif
}