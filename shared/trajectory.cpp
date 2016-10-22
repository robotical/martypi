#include "trajectory.h"

// linear interpolation of trajectory with desired dt
// this function ensures that the original tIn points are explicitly stated in tOut
// it does not (necessarily) result in a uniform dt in tOut

// TODO: Re-Write using iterators rather than popping.
// Also then don't need to copy tIn, maybe rewrite to produce uniform dt on tOut

bool interpTrajectory(data_t tIn, data_t& tOut, float dt) {
  if (tIn.empty())
    return false;

  deque <float> tstart, tend;
  tend = tIn.front();
  tIn.pop_front();

  int numjoints = tend.size() - 1;
  if (DEBUG_MODE) { printf("numjoints: %d\n", numjoints); }

  if (numjoints < 0) return false;

  while (!tIn.empty()) {
    tstart = tend;
    tend = tIn.front();
    tIn.pop_front();

    for (float t = tstart[0]; t < tend[0]; t += dt) {
      deque <float> thisline;
      thisline.push_back(t);
      for (int j = 1; j <= numjoints; j++) {
        thisline.push_back(tstart[j] + (tend[j] - tstart[j]) * (t - tstart[0]) /
                           (tend[0] - tstart[0]));
      }
      tOut.push_back(thisline);
    }
  }
  tOut.push_back(tend);

  return true;
}


void printTrajectory(data_t& traj) {
  if (DEBUG_MODE) {
    printf("trajectory size: %d x %d\n", (int)traj.size(), (int)traj[0].size());

    for (data_t::record_iterator ri = traj.begin(); ri != traj.end(); ri++) {
      for (data_t::field_iterator fi = (*ri).begin(); fi != (*ri).end(); fi++)
      { printf("%2.2f\t", *fi); }
      printf("\n");
    }
  }
  return;
}

bool runTrajectory(martyrobot& robot, data_t& traj) {
  float startTime = gettime();

  data_t::record_iterator ri = traj.begin();
  while (ri != traj.end()) {
    if (gettime() - startTime > (*ri)[0]) {
      if (DEBUG_MODE) {
        printf("specified=%2.4f,\tactual=%2.4f\t", (*ri)[0], gettime() - startTime);
      }
      deque<float> thisline = *ri;
      thisline.pop_front();   // get rid of time from angles
      robot.setServos(thisline);
      ri++;
      robot.printAngles();
    }
  }
  return true;
}

// temp defs
#define WKSMALL 65 // 65 //62 // 75 // 45
#define WKLARGE 125
#define HIPSTEP 30

data_t genStepLeft(martyrobot& robot, int stepLength, int turn, float period,
                   char flags) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  tline[0] = 0.25 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = 0.375 * period;
  tline[1 + R_LKNEE] = WKLARGE;
  tSetpoints.push_back(tline);

  tline[0] = 0.625 * period;
  tline[1 + R_RHIP] = stepLength / 2; tline[1 + R_LHIP] = 0 - stepLength / 2;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = turn;
  tSetpoints.push_back(tline);

  tline[0] = 0.75 * period;
  tline[1 + R_LKNEE] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = period;
  tline[1 + R_RKNEE] = 0; tline[1 + R_LKNEE] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.02);

  return tInterp;
}

data_t genStepRight(martyrobot& robot, int stepLength, int turn, float period,
                    char flags) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  tline[0] = 0.25 * period;
  tline[1 + R_RKNEE] = 0 - WKSMALL; tline[1 + R_LKNEE] = 0 - WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = 0.375 * period;
  tline[1 + R_RKNEE] = 0 - WKLARGE;
  tSetpoints.push_back(tline);

  tline[0] = 0.625 * period;
  tline[1 + R_RHIP] = 0 - stepLength / 2; tline[1 + R_LHIP] = stepLength / 2;
  tline[1 + R_LTWIST] = turn; tline[1 + R_RTWIST] = 0;
  tSetpoints.push_back(tline);

  tline[0] = 0.75 * period;
  tline[1 + R_RKNEE] = 0 - WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = period;
  tline[1 + R_RKNEE] = 0; tline[1 + R_LKNEE] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.02);

  return tInterp;

}

data_t genCelebration(martyrobot& robot, float period) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  tline[0] = 0.1 * period;
  tline[1 + R_RARM] = 200; tline[1 + R_LARM] = 200;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tline[1 + R_EYES] = -20;
  tSetpoints.push_back(tline);


  for (float t = 0.2; t < 1.0; t += 0.2) {
    tline[0] = t * period;
    tline[1 + R_LKNEE] = 50; tline[1 + R_RKNEE] = 50;
    tline[1 + R_RARM] = 200; tline[1 + R_LARM] = 50;
    tline[1 + R_EYES] = -0;
    tSetpoints.push_back(tline);

    tline[0] = (t + 0.1) * period;
    tline[1 + R_LKNEE] = -50; tline[1 + R_RKNEE] = -50;
    tline[1 + R_RARM] = 50; tline[1 + R_LARM] = 200;
    tline[1 + R_EYES] = -40;
    tSetpoints.push_back(tline);
  }


  tline[0] = period;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tline[1 + R_RARM] = 0; tline[1 + R_LARM] = 0;
  tline[1 + R_EYES] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.05);

  return tInterp;
}

data_t genRaisedFootTwistLeft(martyrobot& robot, float period) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  tline[0] = 0.15 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = 0.3 * period;
  tline[1 + R_LKNEE] = WKLARGE;
  tline[1 + R_LHIP] = -150;
  tSetpoints.push_back(tline);

  tline[0] = 0.4 * period;
  tline[1 + R_LTWIST] = -70;
  tSetpoints.push_back(tline);

  tline[0] = 0.5 * period;
  tline[1 + R_LTWIST] = 70;
  tSetpoints.push_back(tline);

  tline[0] = 0.6 * period;
  tline[1 + R_LTWIST] = -70;
  tSetpoints.push_back(tline);

  tline[0] = 0.7 * period;
  tline[1 + R_LTWIST] = 70;
  tSetpoints.push_back(tline);

  tline[0] = 0.85 * period;
  tline[1 + R_LKNEE] = WKSMALL;
  tline[1 + R_LTWIST] = 0;
  tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = period;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.02);

  return tInterp;

}

data_t genRaisedFootTwistRight(martyrobot& robot, float period) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  tline[0] = 0.15 * period;
  tline[1 + R_RKNEE] = -WKSMALL; tline[1 + R_LKNEE] = -WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = 0.3 * period;
  tline[1 + R_RKNEE] = -WKLARGE;
  tline[1 + R_RHIP] = -150;
  tSetpoints.push_back(tline);

  tline[0] = 0.4 * period;
  tline[1 + R_RTWIST] = -70;
  tSetpoints.push_back(tline);

  tline[0] = 0.5 * period;
  tline[1 + R_RTWIST] = 70;
  tSetpoints.push_back(tline);

  tline[0] = 0.6 * period;
  tline[1 + R_RTWIST] = -70;
  tSetpoints.push_back(tline);

  tline[0] = 0.7 * period;
  tline[1 + R_RTWIST] = 70;
  tSetpoints.push_back(tline);

  tline[0] = 0.85 * period;
  tline[1 + R_RKNEE] = -WKSMALL;
  tline[1 + R_RTWIST] = 0;
  tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = period;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.02);

  return tInterp;

}

data_t genKickLeft(martyrobot& robot, float period) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  tline[0] = 0.2 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = 0.375 * period;
  tline[1 + R_LKNEE] = WKLARGE;
  tSetpoints.push_back(tline);

  tline[0] = 0.425 * period;
  tline[1 + R_LHIP] = -150;
  tSetpoints.push_back(tline);

  tline[0] = 0.575 * period;
  tline[1 + R_LHIP] = 0;
  tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = 0.7 * period;
  tline[1 + R_LKNEE] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = period;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.02);

  return tInterp;

}
data_t genKickRight(martyrobot& robot, float period) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  tline[0] = 0.2 * period;
  tline[1 + R_RKNEE] = -WKSMALL; tline[1 + R_LKNEE] = -WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = 0.375 * period;
  tline[1 + R_RKNEE] = -WKLARGE;
  tSetpoints.push_back(tline);

  tline[0] = 0.425 * period;
  tline[1 + R_RHIP] = -150;
  tSetpoints.push_back(tline);

  tline[0] = 0.575 * period;
  tline[1 + R_LHIP] = 0;
  tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = 0.7 * period;
  tline[1 + R_RKNEE] = -WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = period;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.05);
  return tInterp;

}

data_t genGetUp(martyrobot& robot) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  for (std::deque<float>::iterator di = tline.begin(); di != tline.end(); di++)
    *di = 0;
  tline[0] = 1.0;
  tSetpoints.push_back(tline);

  tline[0] = 2.5;
  tline[1 + R_LARM] = -200;
  tline[1 + R_RARM] = 0;
  tSetpoints.push_back(tline);


  tline[0] = 4.0;
  tline[1 + R_LHIP] = -150; tline[1 + R_RHIP] = -150;
  tSetpoints.push_back(tline);

  tline[0] = 6.0;
  tline[1 + R_LKNEE] = 125; tline[1 + R_RKNEE] = -125;
  tSetpoints.push_back(tline);

  tline[0] = 8.0;
  tline[1 + R_RTWIST] = -85; tline[1 + R_LTWIST] = 85;
  tSetpoints.push_back(tline);

  tline[0] = 10.0;
  tline[1 + R_RARM] = 250;
  tSetpoints.push_back(tline);

  tline[0] += 2.0;
  tline[1 + R_RTWIST] = -170; tline[1 + R_LTWIST] = 0;
  tSetpoints.push_back(tline);

  tline[0] += 2.0;
  tline[1 + R_LHIP] = 150;
  tSetpoints.push_back(tline);

  tline[0] += 2.0;
  tline[1 + R_RARM] = 80;
  tSetpoints.push_back(tline);

  tline[0] += 2.0;
  tline[1 + R_RTWIST] = 0;
  tSetpoints.push_back(tline);

  tline[0] += 2.0;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] += 2.0;
  tline[1 + R_RKNEE] = 0; tline[1 + R_LKNEE] = 0;
  tSetpoints.push_back(tline);

  interpTrajectory(tSetpoints, tInterp, 0.02);

  return tInterp;

}

data_t genReturnToZero(martyrobot& robot, float period) {
  data_t tSetpoints, tInterp;
  deque<float> tline(robot.jangles);
  tline.push_front(0.0);
  tSetpoints.push_back(tline);

  // set all joint angles to zero
  for (std::deque<float>::iterator di = tline.begin(); di != tline.end(); di++)
    *di = 0;
  tline[0] = period;
  tSetpoints.push_back(tline);
  interpTrajectory(tSetpoints, tInterp, 0.02);
  return tInterp;
}

bool setPointsLeanLeft(data_t& tSetpoints, int leanAmount, int legLift,
                       float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  //tline[1+R_RHIP] = 0; tline[1+R_LHIP] = 0;
  tline[1 + R_LKNEE] = -leanAmount; tline[1 + R_RKNEE] = -leanAmount;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsLeanRight(data_t& tSetpoints, int leanAmount, int legLift,
                        float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  //tline[1+R_RHIP] = 0; tline[1+R_LHIP] = 0;
  tline[1 + R_LKNEE] = leanAmount; tline[1 + R_RKNEE] = leanAmount;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsLeanForward(data_t& tSetpoints, int leanAmount, float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  tline[1 + R_RHIP] = -leanAmount; tline[1 + R_LHIP] = -leanAmount;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsLeanBackward(data_t& tSetpoints, int leanAmount, float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  tline[1 + R_RHIP] = leanAmount; tline[1 + R_LHIP] = leanAmount;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsLegsZero(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsCrossLeftLeg(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  tline[1 + R_LHIP] = -75; tline[1 + R_RHIP] = 75;
  tline[1 + R_LKNEE] = 20; tline[1 + R_RKNEE] = -20;
  tline[1 + R_RTWIST] = -40; tline[1 + R_LTWIST] = 80;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsCrossRightLeg(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  tline[1 + R_LHIP] = 75; tline[1 + R_RHIP] = -75;
  tline[1 + R_LKNEE] = 20; tline[1 + R_RKNEE] = -20;
  tline[1 + R_LTWIST] = 40; tline[1 + R_RTWIST] = -80;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsLegsApart(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());

  tline[0] += period;
  tline[1 + R_LKNEE] = 60; tline[1 + R_RKNEE] = -60;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsKickOutLeft(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.2 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKSMALL;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.275 * period;
  tline[1 + R_LKNEE] = WKLARGE;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.5 * period;
  tline[1 + R_RHIP] = 0;
  tline[1 + R_LHIP] = -150;
  tline[1 + R_LTWIST] = 80;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.675 * period;
  tline[1 + R_LHIP] = 0;
  tline[1 + R_RHIP] = 0;
  tline[1 + R_LTWIST] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.725 * period;
  tline[1 + R_LKNEE] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsKickOutRight(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.2 * period;
  tline[1 + R_RKNEE] = -WKSMALL; tline[1 + R_LKNEE] = -WKSMALL;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.275 * period;
  tline[1 + R_RKNEE] = -WKLARGE;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.5 * period;
  tline[1 + R_LHIP] = 0;
  tline[1 + R_RHIP] = -150;
  tline[1 + R_RTWIST] = -80;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.675 * period;
  tline[1 + R_LHIP] = 0;
  tline[1 + R_RHIP] = 0;
  tline[1 + R_RTWIST] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.725 * period;
  tline[1 + R_RKNEE] = -WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsFlickRight(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.5 * period;
  tline[1 + R_RKNEE] = -WKSMALL; tline[1 + R_LKNEE] = 0;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tline[1 + R_RHIP] = -50; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_RKNEE] = 0;
  tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsCircleACW(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_RKNEE] = -WKSMALL; tline[1 + R_LKNEE] = -WKSMALL;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.5 * period;
  tline[1 + R_RKNEE] = 0; tline[1 + R_LKNEE] = 0;
  tline[1 + R_RHIP] = 60; tline[1 + R_LHIP] = 60;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKSMALL;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_RKNEE] = 0; tline[1 + R_LKNEE] = 0;
  tline[1 + R_RHIP] = -60; tline[1 + R_LHIP] = -60;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsCircleCW(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKSMALL;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.5 * period;
  tline[1 + R_RKNEE] = 0; tline[1 + R_LKNEE] = 0;
  tline[1 + R_RHIP] = 60; tline[1 + R_LHIP] = 60;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_RKNEE] = -WKSMALL; tline[1 + R_LKNEE] = -WKSMALL;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_RKNEE] = 0; tline[1 + R_LKNEE] = 0;
  tline[1 + R_RHIP] = -60; tline[1 + R_LHIP] = -60;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsTapFR(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_LKNEE] = -WKSMALL; tline[1 + R_RKNEE] = -WKLARGE;
  tline[1 + R_LHIP] = 0; tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_RHIP] = -WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsTapMR(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_LKNEE] = -WKSMALL; tline[1 + R_RKNEE] = -WKLARGE;
  tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_RKNEE] = -WKSMALL;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsTapBR(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_LKNEE] = -WKSMALL; tline[1 + R_RKNEE] = -WKLARGE;
  tline[1 + R_LHIP] = 0; tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_RHIP] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsTapFL(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKLARGE;
  tline[1 + R_LHIP] = 0; tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_LHIP] = -WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_LKNEE] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsTapML(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKLARGE;
  tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_LKNEE] = WKSMALL;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsTapBL(data_t& tSetpoints, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + 0.25 * period;
  tline[1 + R_RKNEE] = WKSMALL; tline[1 + R_LKNEE] = WKLARGE;
  tline[1 + R_LHIP] = 0; tline[1 + R_RHIP] = 0;
  tSetpoints.push_back(tline);

  tline[0] = startTime + 0.75 * period;
  tline[1 + R_LHIP] = WKSMALL;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_LKNEE] = 0;
  tSetpoints.push_back(tline);

  return 1;
}


bool setPointsArmsUp(data_t& tSetpoints, float amountRight, float amountLeft,
                     float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + period;
  tline[1 + R_RARM] = amountRight; tline[1 + R_LARM] = amountLeft;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsLeftArmUp(data_t& tSetpoints, float amount, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + period;
  tline[1 + R_LARM] = amount;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsRightArmUp(data_t& tSetpoints, float amount, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + period;
  tline[1 + R_RARM] = amount;
  tSetpoints.push_back(tline);

  return 1;
}

bool setPointsEyes(data_t& tSetpoints, float targetPos, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  tline[0] = startTime + period;
  tline[1 + R_EYES] = targetPos;
  tSetpoints.push_back(tline);

  return 1;
}

// function to combine some joints from one trajectory with some joints from another
// e.g. for combining leg and arm movements
// treats the time code in the same way as a joint
//    i.e. THE dt FOR BOTH TRAJECTORIES MUST BE THE SAME!
data_t combineTrajectories(data_t& t1, data_t& t2, vector<bool> which) {
  data_t traj;
  data_t::record_iterator ni2 = t2.begin();
  for (data_t::record_iterator ni1 = t1.begin(); ni1 != t1.end(); ni1++) {
    deque<float> tline;
    for (unsigned int ji = 0; ji < which.size(); ji++) {
      float tfield = !which[ji] ? (*ni1)[ji] : (*ni2)[ji];
      tline.push_back(tfield);
    }
    traj.push_back(tline);
    if (ni2 != t2.end())
      ni2++;
    if (ni2 == t2.end())
      ni2--;
  }
  return traj;
}

data_t combineLegsArmsEyes(data_t& legs, data_t& arms, data_t& eyes) {
  data_t tlegsArms, genTraj;
  vector<bool> ji(1 + R_NUMJOINTS, 0);
  ji[1 + R_RARM] = 1; ji[1 + R_LARM] = 1;
  tlegsArms = combineTrajectories(legs, arms, ji);
  ji[1 + R_RARM] = 0; ji[1 + R_LARM] = 0;
  ji[1 + R_EYES] = 1;
  genTraj = combineTrajectories(tlegsArms, eyes, ji);
  return genTraj;
}


int hipToBeSquare(martyrobot& robot, int robotID) {
#define BEAT  0.4255

  printf("starting\n");
  robot.init();

  data_t tSetpoints, genTraj;

  data_t tLegs, tArms, tEyes;
  data_t tiLegs, tiArms, tiEyes;
  deque<float> tline(robot.jangles);
  tline.push_front(0);
  tLegs.push_back(tline);
  tArms.push_back(tline);
  tEyes.push_back(tline);

  int polarity = 0;
  if (robotID == 0) {
    polarity = -1;
  } else if (robotID == 2) {
    polarity = 1;
  }

  // leaning forward and backward
  for (int i = 0; i < 3; i++) {
    setPointsLeanForward(tLegs, 0, BEAT / 2);
    setPointsArmsUp(tArms, 0, 0, BEAT / 2);
    setPointsEyes(tEyes, 0, BEAT / 2);

    setPointsLeanForward(tLegs, 45 * polarity, BEAT / 2);
    setPointsArmsUp(tArms, -200, 200, BEAT / 2);
    setPointsEyes(tEyes, 30 * polarity, BEAT / 2);
    setPointsLeanForward(tLegs, 45 * polarity, BEAT);
    setPointsArmsUp(tArms, -200, 200, BEAT);
    setPointsEyes(tEyes, 30 * polarity, BEAT);

    setPointsLeanForward(tLegs, 0, BEAT / 2);
    setPointsArmsUp(tArms, 0, 0, BEAT / 2);
    setPointsEyes(tEyes, 0, BEAT / 2);
    setPointsLeanForward(tLegs, -45 * polarity, BEAT / 2);
    setPointsArmsUp(tArms, 200, -200, BEAT / 2);
    setPointsEyes(tEyes, -30 * polarity, BEAT / 2);
    setPointsLeanForward(tLegs, -45 * polarity, BEAT);
    setPointsArmsUp(tArms, 200, -200, BEAT);
    setPointsEyes(tEyes, -30 * polarity, BEAT);
  }

  // staggered center, with eyes wide, then eyes together all at the same time
  if (robotID == 0) {
    setPointsLeanForward(tLegs, 0, BEAT);
    setPointsEyes(tEyes, 30, BEAT);
    setPointsLeanForward(tLegs, 0, BEAT * 2);
    setPointsEyes(tEyes, 30, BEAT * 2);
    setPointsArmsUp(tArms, 0, 0, BEAT * 3);
  } else if (robotID == 1) {
    setPointsLeanForward(tLegs, -45 * polarity, BEAT);
    setPointsEyes(tEyes, 0, BEAT);
    setPointsArmsUp(tArms, -200 * polarity, 200 * polarity, BEAT);
    setPointsLeanForward(tLegs, 0, BEAT);
    setPointsEyes(tEyes, 30, BEAT);
    setPointsArmsUp(tArms, 0, 0, BEAT);
    setPointsLeanForward(tLegs, 0, BEAT);
    setPointsEyes(tEyes, 30, BEAT);
    setPointsArmsUp(tArms, 0, 0, BEAT);
  } else if (robotID == 2) {
    setPointsLeanForward(tLegs, -45 * polarity, BEAT * 2);
    setPointsEyes(tEyes, 30, BEAT * 2);
    setPointsLeanForward(tLegs, 0, BEAT);
    setPointsEyes(tEyes, 30, BEAT);
    setPointsArmsUp(tArms, 0, 0, BEAT * 3);
  }
  setPointsLeanForward(tLegs, 0, BEAT);
  setPointsEyes(tEyes, 0, BEAT);
  setPointsArmsUp(tArms, 0, 0, BEAT);

  // lean left, pause, lean right, pause
  for (int i = 0; i < 3; i++) {
    // center
    setPointsLeanLeft(tLegs, 0, 0, BEAT / 2);
    setPointsArmsUp(tArms, 0, 0, BEAT / 2);
    // lean left and pause
    setPointsLeanLeft(tLegs, 45, 0, BEAT / 2);
    setPointsArmsUp(tArms, 0, 200, BEAT / 2);
    setPointsLeanLeft(tLegs, 45, 0, BEAT);
    setPointsArmsUp(tArms, 0, 200, BEAT);

    //lean right and pause
    setPointsLeanLeft(tLegs, 0, 0, BEAT / 2);
    setPointsArmsUp(tArms, 0, 0, BEAT / 2);
    setPointsLeanRight(tLegs, 45, 0, BEAT / 2);
    setPointsArmsUp(tArms, 200, 0, BEAT / 2);
    setPointsLeanRight(tLegs, 45, 0, BEAT);
    setPointsArmsUp(tArms, 200, 0, BEAT);
  }
  setPointsEyes(tEyes, 0, BEAT * 12);

  // staggered centering, then all lean left at same time
  if (robotID == 0) {
    setPointsLeanLeft(tLegs, 0, 0, BEAT);
    setPointsLeanLeft(tLegs, 0, 0, BEAT * 2);
  } else if (robotID == 1) {
    setPointsLeanRight(tLegs, 45, 0, BEAT);
    setPointsLeanLeft(tLegs, 0, 0, BEAT);
    setPointsLeanLeft(tLegs, 0, 0, BEAT);
  } else if (robotID == 2) {
    setPointsLeanRight(tLegs, 45, 0, BEAT * 2);
    setPointsLeanLeft(tLegs, 0, 0, BEAT);
  }
  setPointsLeanLeft(tLegs, 45, 0, BEAT);
  setPointsArmsUp(tArms, 0, 0, BEAT * 4);
  setPointsEyes(tEyes, 0, BEAT * 4);

  //


  interpTrajectory(tLegs, tiLegs, 0.05);
  interpTrajectory(tArms, tiArms, 0.05);
  interpTrajectory(tEyes, tiEyes, 0.05);
  genTraj = combineLegsArmsEyes(tiLegs, tiArms, tiEyes);

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
    if (robotID == 0) {
      setPointsKickOutLeft(tLegs, BEAT * 4);
    } else if (robotID == 1) {
      setPointsLeanBackward(tLegs, 45, BEAT * 3);
      setPointsLeanBackward(tLegs, 0, BEAT);
    } else {
      setPointsKickOutRight(tLegs, BEAT * 4);
    }

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

  setPointsLegsZero(tLegs, BEAT);
  setPointsArmsUp(tArms, 0, 0, BEAT);

  vector<bool> ji(1 + R_NUMJOINTS, 0);
  ji[1 + R_RARM] = 1; ji[1 + R_LARM] = 1;
  interpTrajectory(tLegs, tiLegs, 0.05);
  interpTrajectory(tArms, tiArms, 0.05);
  genTraj = combineTrajectories(tiLegs, tiArms, ji);

  runTrajectory(robot, genTraj);

  return 1;
}

bool setPointsSkateLeft(data_t& tSetpoints, float amount, float period) {
  deque <float> tline(tSetpoints.back());
  float startTime = tline[0];

  // lean right
  tline[0] = startTime + 0.2 * period;
  tline[1 + R_RKNEE] = 55; tline[1 + R_LKNEE] = 55;
  tline[1 + R_LTWIST] = 0; tline[1 + R_RTWIST] = 0;
  tline[1 + R_RHIP] = 0; tline[1 + R_LHIP] = 0;
  tSetpoints.push_back(tline);

  // lift left foot
  tline[0] = startTime + 0.275 * period;
  tline[1 + R_LKNEE] = 75;
  tSetpoints.push_back(tline);

  // right leg backward, left forward and twist
  tline[0] = startTime + 0.5 * period;
  tline[1 + R_RHIP] = -30;
  tline[1 + R_LHIP] = -50;
  tline[1 + R_LTWIST] = 30;
  tSetpoints.push_back(tline);

  // left leg swing back
  tline[0] = startTime + 0.675 * period;
  tline[1 + R_LHIP] = 0;
  //tline[1+R_RHIP] = 0;
  //tline[1+R_LTWIST] = 0;
  tSetpoints.push_back(tline);

  // left leg swing back, pt 2
  tline[0] = startTime + 0.725 * period;
  tline[1 + R_LHIP] = 50;
  tline[1 + R_RHIP] = 0;
  tline[1 + R_LTWIST] = 0;
  //tline[1+R_LKNEE] = 75;
  tSetpoints.push_back(tline);

  tline[0] = startTime + period;
  tline[1 + R_LHIP] = 0;
  tline[1 + R_LKNEE] = 0; tline[1 + R_RKNEE] = 0;
  tSetpoints.push_back(tline);

  return 1;
}

int rollerSkate(martyrobot& robot) {
  data_t tSetpoints, genTraj;

  data_t tLegs, tArms, tEyes;
  data_t tiLegs, tiArms, tiEyes;
  deque<float> tline(robot.jangles);
  tline.push_front(0);
  tLegs.push_back(tline);
  tArms.push_back(tline);
  tEyes.push_back(tline);

  setPointsLegsZero(tLegs, 1.0);
  setPointsSkateLeft(tLegs, 0, 3.0);

  interpTrajectory(tLegs, tiLegs, 0.05);

  runTrajectory(robot, tiLegs);

  return 1;
}
