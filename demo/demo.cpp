#include <iostream>
#include <unistd.h>
#include <math.h>
//#include <linux/i2c-dev.h>
#include "i2c.h"

#include <string>
#include <sstream>
#include <iostream>
#include <fstream>
#include <deque>
#include <time.h>
#include "martyrobot.h"
#include "data_t.h"
#include "trajectory.h"
#include "utils.h"

using namespace std;

int main() {
  INFO("DEMO: Starting\n");
  martyrobot robot;
  robot.init();

  float alpha = 0;

#define DDELTA  17500
  deque<float> setpos(R_NUMJOINTS, 0);

  // Default Arm positions
  robot.setServo(R_EYES, EYESNORMAL);
  sleep(1);
  robot.setServo(R_EYES, EYESANGRY);
  sleep(1);
  robot.setServo(R_EYES, EYESEXCITED);
  sleep(1);
  robot.setServo(R_EYES, EYESWIDE);
  sleep(1);

  // Test eyes
  for (alpha = 0; alpha < 6.28; alpha += 0.1) {
    robot.setServo(R_EYES, EYESANGRY * sin(alpha));;
    usleep(DDELTA);
  }

  // Arms Swing Together
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    robot.setServo(R_RARM, 100 * sin(alpha));
    robot.setServo(R_LARM, 100 * sin(alpha));
    usleep(DDELTA);
  }

  // Arms Swing Opposed
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    robot.setServo(R_RARM, 100 * sin(alpha));
    robot.setServo(R_LARM, -100 * sin(alpha));
    usleep(DDELTA);
  }

  // Forward and Backward
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    setpos[R_RHIP] = 45 * sin(alpha); setpos[R_LHIP] = setpos[R_RHIP];
    robot.setServos(setpos);
    usleep(DDELTA);
  }

  // Side to Side
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    setpos[R_RKNEE] = 45 * sin(alpha); setpos[R_LKNEE] = setpos[R_RKNEE];
    robot.setServos(setpos);
    usleep(DDELTA);
  }

  // Feet Twist in same direction
  setpos[R_RHIP] = 0; setpos[R_RKNEE] = 0; setpos[R_LHIP] = 0;
  setpos[R_LKNEE] = 0;
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    setpos[R_RTWIST] = 45 * sin(alpha); setpos[R_LTWIST] = setpos[R_RTWIST];
    robot.setServos(setpos);

    usleep(DDELTA);
  }

  // Feet Twist in opposite direction
  for (alpha = 0; alpha < 12.56; alpha += 0.1) {
    setpos[R_RTWIST] = 20 * sin(alpha); setpos[R_LTWIST] = 0 - setpos[R_RTWIST];
    robot.setServos(setpos);
    usleep(DDELTA);
  }

  // Test Left Leg
  data_t genTraj;
  genTraj = genRaisedFootTwistLeft(robot, 4.0);
  runTrajectory(robot, genTraj);
  genTraj.clear();

  // Test Right Leg
  genTraj = genRaisedFootTwistRight(robot, 4.0);
  runTrajectory(robot, genTraj);
  genTraj.clear();

  // Walk about
  for (int numstep = 0; numstep < 1; numstep++) {
    genTraj = genStepLeft(robot, 0, 25, 1.25, 0);
    runTrajectory(robot, genTraj);
    genTraj.clear();

    genTraj = genStepRight(robot, 0, 25, 1.25, 0);
    runTrajectory(robot, genTraj);
    genTraj.clear();
  }

  for (int numstep = 0; numstep < 2; numstep++) {
    genTraj = genStepLeft(robot, 80, 0, 1.5, 0);
    runTrajectory(robot, genTraj);
    genTraj.clear();

    genTraj = genStepRight(robot, 80, 0, 1.5, 0);
    runTrajectory(robot, genTraj);
    genTraj.clear();
  }

  // Test a Left Kick
  genTraj = genKickLeft(robot, 2.0);
  runTrajectory(robot, genTraj);
  genTraj.clear();

  // Celebrate!
  genTraj = genCelebration(robot, 4.0);
  runTrajectory(robot, genTraj);
  genTraj.clear();

  return 1;


}
