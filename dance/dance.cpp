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
  INFO("DANCE: Starting\n");
  martyrobot robot;
  robot.init();

  data_t tSetpoints, genTraj;
  data_t tLegs, tArms;

  // Dance 1
  deque<float> tline(robot.jangles);
  tline.push_front(0);
  tLegs.push_back(tline);
  tArms.push_back(tline);

  setPointsCircleCW(tLegs, 2.0);
  setPointsCircleCW(tLegs, 2.0);
  setPointsCircleCW(tLegs, 2.0);
  setPointsCircleCW(tLegs, 2.0);
  setPointsArmsUp(tArms, 150, 150, 1.0);
  setPointsArmsUp(tArms, 100, 200, 1.0);
  setPointsArmsUp(tArms, 200, 100, 1.0);
  setPointsArmsUp(tArms, 100, 200, 1.0);
  setPointsArmsUp(tArms, 200, 100, 1.0);
  setPointsArmsUp(tArms, 100, 200, 1.0);
  setPointsArmsUp(tArms, 200, 100, 1.0);
  setPointsArmsUp(tArms, 50, 50, 1.0);

  data_t tiLegs, tiArms;
  interpTrajectory(tLegs, tiLegs, 0.05);
  interpTrajectory(tArms, tiArms, 0.05);
  vector<bool> ji(1 + R_NUMJOINTS, 0);
  ji[1 + R_RARM] = 1; ji[1 + R_LARM] = 1;
  genTraj = combineTrajectories(tiLegs, tiArms, ji);
  printTrajectory(tiArms);
  printTrajectory(genTraj);
  runTrajectory(robot, genTraj);

  // Dance 2
  /*
  genTraj = genStepLeft(robot, 50, 0, 1.5, 0);
  runTrajectory(robot, genTraj);
  genTraj = genStepRight(robot, 50, 0, 1.5, 0);
  runTrajectory(robot, genTraj);
  genTraj = genKickLeft(robot, 2.0);
  runTrajectory(robot, genTraj);
  genTraj = genKickRight(robot, 2.0);
  runTrajectory(robot, genTraj);
  genTraj = genRaisedFootTwistLeft(robot, 4.0);
  runTrajectory(robot, genTraj);
  genTraj = genRaisedFootTwistRight(robot, 4.0);
  runTrajectory(robot, genTraj);
  genTraj = genCelebration(robot, 4.0);
  runTrajectory(robot, genTraj);
  deque<float> tline2(robot.jangles);
  tline2.push_front(0);
  tSetpoints.push_back(tline2);
  setPointsLeanLeft(tSetpoints, 40, 0, 1.0);
  setPointsLeanRight(tSetpoints, 40, 0, 1.0);
  setPointsLeanForward(tSetpoints, 40, 1.0);
  setPointsLeanBackward(tSetpoints, 40, 1.0);
  setPointsLegsZero(tSetpoints, 1.0);
  setPointsCrossLeftLeg(tSetpoints, 1.5);
  setPointsCrossRightLeg(tSetpoints, 1.5);
  setPointsLegsApart(tSetpoints, 2.0);
  setPointsKickOutLeft(tSetpoints, 2.0);
  setPointsKickOutRight(tSetpoints, 1.5);
  setPointsFlickRight(tSetpoints, 2.0);
  setPointsCircleACW(tSetpoints, 2.0);
  setPointsCircleCW(tSetpoints, 2.0);

  genTraj.clear();
  interpTrajectory(tSetpoints, genTraj, 0.05);
  runTrajectory(robot, genTraj);
  */

  // Return to Home Positions
  genTraj.clear();
  genTraj = genReturnToZero(robot, 1.0);
  runTrajectory(robot, genTraj);

  return 1;
}
