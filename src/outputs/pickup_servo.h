//
// Created by elise on 4/07/2026.
//

#ifndef PICKUP_SERVO_H
#define PICKUP_SERVO_H

void pickup_servo_init();
void pickup_servo_exe();

static void moveCraneTo(int target, int speed);

void craneVertical();
void craneHorizontal();
void craneDrop();
void craneIdle();

#endif