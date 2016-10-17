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

#define BEAT  0.4255


int main() {
  printf("starting\n");
  martyrobot robot;
  robot.init();

  data_t tSetpoints, genTraj;

  data_t tLegs, tArms;
  data_t tiLegs, tiArms;
  deque<float> tline(robot.jangles);
  tline.push_front(0);
  tLegs.push_back(tline);
  tArms.push_back(tline);

  for (int i = 0; i < 7; i++) {
    setPointsLeanLeft(tLegs, 0, 0, BEAT / 2);
    setPointsLeanLeft(tLegs, 45, 0, BEAT / 2);
    setPointsLeanLeft(tLegs, 45, 0, BEAT);
    setPointsLeanLeft(tLegs, 0, 0, BEAT / 2);
    setPointsLeanRight(tLegs, 45, 0, BEAT / 2);
    setPointsLeanRight(tLegs, 45, 0, BEAT);
  }

  for (int i = 0; i < 7; i++) {
    setPointsArmsUp(tArms, 0, 200, BEAT * 2);
    setPointsArmsUp(tArms, 200, 0, BEAT * 2);
  }

  setPointsLeanLeft(tLegs, 0, 0, BEAT);
  setPointsLeanLeft(tLegs, 0, 0, BEAT * 2);
  setPointsLeanLeft(tLegs, 45, 0, BEAT);
  setPointsArmsUp(tArms, 0, 0, BEAT * 4);


  interpTrajectory(tLegs, tiLegs, 0.05);
  interpTrajectory(tArms, tiArms, 0.05);
  vector<bool> ji(1 + R_NUMJOINTS, 0);
  ji[1 + R_RARM] = 1; ji[1 + R_LARM] = 1;
  genTraj = combineTrajectories(tiLegs, tiArms, ji);

  runTrajectory(robot, genTraj);

  tline = genTraj.back();
  tline[0] = 0;

  tLegs.clear();
  tArms.clear();
  tLegs.push_back(tline);
  tArms.push_back(tline);
  tiArms.clear();
  tiLegs.clear();
  genTraj.clear();

  setPointsTapFR(tLegs, BEAT * 2);
  setPointsTapMR(tLegs, BEAT * 2);
  setPointsTapBR(tLegs, BEAT * 2);
  setPointsTapMR(tLegs, BEAT * 2);
  setPointsTapFR(tLegs, BEAT * 2);
  setPointsTapMR(tLegs, BEAT * 2);
  setPointsTapMR(tLegs, BEAT * 2);
  setPointsLeanLeft(tLegs, 0, 0, BEAT);
  setPointsLeanRight(tLegs, 45, 0, BEAT);
  setPointsTapFL(tLegs, BEAT * 2);
  setPointsTapML(tLegs, BEAT * 2);
  setPointsTapBL(tLegs, BEAT * 2);
  setPointsTapML(tLegs, BEAT * 2);
  setPointsTapFL(tLegs, BEAT * 2);
  setPointsTapML(tLegs, BEAT * 2);
  setPointsTapML(tLegs, BEAT * 2);
  setPointsLeanLeft(tLegs, 0, 0, BEAT);
  setPointsLeanLeft(tLegs, 0, 0, BEAT);

  interpTrajectory(tLegs, tiLegs, 0.05);
  runTrajectory(robot, tiLegs);

  tline = tLegs.back();
  tline[0] = 0;

  tLegs.clear();
  tArms.clear();
  tLegs.push_back(tline);
  tArms.push_back(tline);
  tiArms.clear();
  tiLegs.clear();
  genTraj.clear();

  for (int i = 0; i < 2; i++) {
    setPointsLeanLeft(tLegs, 45, 0, BEAT * 4);
    setPointsArmsUp(tArms, 0, 200, BEAT * 4);
    setPointsLeanRight(tLegs, 45, 0, BEAT * 4);
    setPointsArmsUp(tArms, 200, 0, BEAT * 4);
    setPointsLeanLeft(tLegs, 45, 0, BEAT * 2);
    setPointsArmsUp(tArms, 0, 200, BEAT * 2);
    setPointsLeanRight(tLegs, 45, 0, BEAT * 2);
    setPointsArmsUp(tArms, 200, 0, BEAT * 2);
    setPointsKickOutLeft(tLegs, BEAT * 4);
    setPointsArmsUp(tArms, 0, 0, BEAT * 4);

    //for (int j=0; j<2; j++){
    //  setPointsLeanLeft(tLegs, 45, 0, BEAT);
    //  setPointsArmsUp(tArms, 0, 200, BEAT);
    //  setPointsLeanRight(tLegs, 45, 0, BEAT);
    //  setPointsArmsUp(tArms, 200, 0, BEAT);
    //}
  }

  for (int i = 0; i < 4; i++) {
    setPointsCircleCW(tLegs, BEAT * 4);
    setPointsArmsUp(tArms, 0, 0, BEAT * 4);
  }

  interpTrajectory(tLegs, tiLegs, 0.05);
  interpTrajectory(tArms, tiArms, 0.05);
  genTraj = combineTrajectories(tiLegs, tiArms, ji);

  runTrajectory(robot, genTraj);

  return 1;
  setPointsArmsUp(tArms, 150, 150, 1.0);
  setPointsArmsUp(tArms, 100, 200, 1.0);
  setPointsArmsUp(tArms, 200, 100, 1.0);
  setPointsArmsUp(tArms, 100, 200, 1.0);
  setPointsArmsUp(tArms, 200, 100, 1.0);
  setPointsArmsUp(tArms, 100, 200, 1.0);
  setPointsArmsUp(tArms, 200, 100, 1.0);
  setPointsArmsUp(tArms, 50, 50, 1.0);

  interpTrajectory(tLegs, tiLegs, 0.05);
  interpTrajectory(tArms, tiArms, 0.05);
  //vector<bool> ji(1+R_NUMJOINTS, 0);
  //ji[1+R_RARM] = 1; ji[1+R_LARM] = 1;
  genTraj = combineTrajectories(tiLegs, tiArms, ji);
  printTrajectory(tiArms);
  printTrajectory(genTraj);

  runTrajectory(robot, genTraj);

  genTraj.clear();

  genTraj = genCelebration(robot, 4.0);
  runTrajectory(robot, genTraj);

  genTraj.clear();
  genTraj = genRaisedFootTwistLeft(robot, 4.0);
  runTrajectory(robot, genTraj);
  genTraj.clear();

  genTraj = genRaisedFootTwistRight(robot, 4.0);
  runTrajectory(robot, genTraj);
  genTraj.clear();

  /*
  // full runthrough
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
  deque<float> tline(robot.jangles);
  tline.push_front(0);
  tSetpoints.push_back(tline);
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

  /*
  deque<float> tline(robot.jangles);
  tline.push_front(0);
  tSetpoints.push_back(tline);

  setPointsCircleCW(tSetpoints, 2.0);
  setPointsCircleCW(tSetpoints, 2.0);
  setPointsCircleACW(tSetpoints, 2.0);
  setPointsCircleACW(tSetpoints, 2.0);
  setPointsCircleACW(tSetpoints, 1.5);
  setPointsCircleACW(tSetpoints, 1.0);
  interpTrajectory(tSetpoints, genTraj, 0.05);
  runTrajectory(robot, genTraj);
  */
  genTraj.clear();
  genTraj = genReturnToZero(robot, 1.0);
  runTrajectory(robot, genTraj);

  return 1;


  //manualCalibration(robot);

  //return 1;



}
