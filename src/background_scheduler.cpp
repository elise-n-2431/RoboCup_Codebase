/* Handles the background scheduling of tasks, based around a round robin scheduler, using TaskScheduler.
 * The ISRs are called when sensors are triggered.
*/

#include <TaskScheduler.h>




// Task period Definitions
#define IR_READ_TASK_PERIOD                 40
#define COLOUR_READ_TASK_PERIOD             40


// Task execution amount definitions
// -1 means indefinitely
#define IR_READ_TASK_NUM_EXECUTE           -1
#define COLOUR_READ_TASK_NUM_EXECUTE       -1

// Task definitions
Task tRead_infrared(IR_READ_TASK_PERIOD,         IR_READ_TASK_NUM_EXECUTE,        &read_infrared);
Task tRead_colour(COLOUR_READ_TASK_PERIOD,       COLOUR_READ_TASK_NUM_EXECUTE,    &read_colour);

Scheduler taskManager;

void setup() {
    // Most initialization handled in main.cpp
}