#ifndef __ARCHIE_TRAJECTORY
#define __ARCHIE_TRAJECTORY

#include "data_t.h"
#include "martyrobot.h"
#include "utils.h"
#include <deque>
#include <vector>

#define TRAJ_FROMZERO   0x01
#define TRAJ_TOZERO   0x02

bool interpTrajectory(data_t tIn, data_t& tOut, float dt);
void printTrajectory(data_t& traj);
bool runTrajectory(martyrobot& robot, data_t& traj);
data_t genStepLeft(martyrobot& robot, int stepLength, int turn, float period,
                   char flags);
data_t genStepRight(martyrobot& robot, int stepLength, int turn, float period,
                    char flags);
data_t genKickLeft(martyrobot& robot, float period);
data_t genKickRight(martyrobot& robot, float period);
data_t genGetUp(martyrobot& robot);
data_t genRaisedFootTwistLeft(martyrobot& robot, float period);
data_t genRaisedFootTwistRight(martyrobot& robot, float period);
data_t genCelebration(martyrobot& robot, float period);
data_t genReturnToZero(martyrobot& robot, float period);
bool setPointsLeanLeft(data_t& tSetpoints, int leanAmount, int legLift,
                       float period);
bool setPointsLeanRight(data_t& tSetpoints, int leanAmount, int legLift,
                        float period);
bool setPointsLeanForward(data_t& tSetpoints, int leanAmount, float period);
bool setPointsLeanBackward(data_t& tSetpoints, int leanAmount, float period);
bool setPointsLegsZero(data_t& tSetpoints, float period);
bool setPointsCrossLeftLeg(data_t& tSetpoints, float period);
bool setPointsCrossRightLeg(data_t& tSetpoints, float period);
bool setPointsLegsApart(data_t& tSetpoints, float period);
bool setPointsKickOutLeft(data_t& tSetpoints, float period);
bool setPointsKickOutRight(data_t& tSetpoints, float period);
bool setPointsFlickRight(data_t& tSetpoints, float period);
bool setPointsCircleACW(data_t& tSetpoints, float period);
bool setPointsCircleCW(data_t& tSetpoints, float period);
bool setPointsTapFR(data_t& tSetpoints, float period);
bool setPointsTapMR(data_t& tSetpoints, float period);
bool setPointsTapBR(data_t& tSetpoints, float period);
bool setPointsTapFL(data_t& tSetpoints, float period);
bool setPointsTapML(data_t& tSetpoints, float period);
bool setPointsTapBL(data_t& tSetpoints, float period);

bool setPointsArmsUp(data_t& tSetpoints, float amountRight, float amountLeft,
                     float period);
bool setPointsLeftArmUp(data_t& tSetpoints, float amount, float period);
bool setPointsRightArmUp(data_t& tSetpoints, float amount, float period);

bool setPointsEyes(data_t& tSetpoints, float targetPos, float period);

data_t combineTrajectories(data_t& t1, data_t& t2, vector<bool> ti);
data_t combineLegsArmsEyes(data_t& legs, data_t& arms, data_t& eyes);

int hipToBeSquare(martyrobot& robot, int robotID);
bool setPointsSkateLeft(data_t& tSetpoints, float amount, float period);
int rollerSkate(martyrobot& robot);

#endif
