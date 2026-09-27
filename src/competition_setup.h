#ifndef COMPETITION_SETUP_H
#define COMPETITION_SETUP_H

enum StartingBase
{
    LOWER_BASE,
    UPPER_BASE
};


constexpr StartingBase STARTING_BASE =
    LOWER_BASE;



struct StartingWeight
{
    float x;
    float y;
};


static const StartingWeight STARTING_WEIGHTS[] =
{
    {2450.0f, 300.0f},
    {2450.0f, 1200.0f},/*,
    {3900.0f, 900.0f}*/
};


constexpr int NUM_STARTING_WEIGHTS =
    sizeof(STARTING_WEIGHTS) /
    sizeof(STARTING_WEIGHTS[0]);



constexpr float COMPETITION_HOME_X =
    300.0f;

constexpr float COMPETITION_HOME_Y =
    STARTING_BASE == LOWER_BASE
        ? 300.0f
        : 2100.0f;

constexpr float COMPETITION_START_X =
    COMPETITION_HOME_X;

constexpr float COMPETITION_START_Y =
    COMPETITION_HOME_Y;

constexpr float COMPETITION_START_HEADING =
    0.0f;


#endif