#include "utils.h"

int manualCalibration(martyrobot& robot) {
  char c;
  bool saving(false);
  bool disabled(false);
  map<string, int> joints;

  joints["LHIP"] = LHIPZERO;
  joints["LTWIST"] = LTWISTZERO;
  joints["LKNEE"] = LKNEEZERO;
  joints["RHIP"] = RHIPZERO;
  joints["RTWIST"] = RTWISTZERO;
  joints["RKNEE"] = RKNEEZERO;
  joints["LARM"] = LARMZERO;
  joints["RARM"] = RARMZERO;
  joints["EYES"] = EYESZERO;
  joints["AUX1"] = AUX1ZERO;
  joints["AUX2"] = AUX2ZERO;

  DEBUG("Starting manual calibration...\n");
  while (1) {
    CLEAR();
    INFO("q-a\tw-s\te-d\tr-f\tt-g\ty-h\tz-x\tc-v\tu-j\ti-k\to-l\tENTER to save\n");
    INFO("LHIP\tLTWIST\tLKNEE\tRHIP\tRTWIST\tRKNEE\tLARM\tRARM\tEYES\tAUX1\tAUX2\t");
    if (disabled) { INFO("n-Disabled\n") } else { INFO("n-Enabled\n"); }
    INFO(joints["LHIP"] << "\t" << joints["LTWIST"] << "\t" << joints["LKNEE"] <<
         "\t" << joints["RHIP"] << "\t" << joints["RTWIST"] << "\t" << joints["RKNEE"]
         << "\t" << joints["LARM"] << "\t" << joints["RARM"] << "\t" << joints["EYES"]
         << "\t" << joints["AUX1"] << "\t" << joints["AUX2"] << endl);
    if (!disabled) {
      robot.setServoJointPulse(R_LHIP, joints["LHIP"]);
      robot.setServoJointPulse(R_LTWIST, joints["LTWIST"]);
      robot.setServoJointPulse(R_LKNEE, joints["LKNEE"]);
      robot.setServoJointPulse(R_RHIP, joints["RHIP"]);
      robot.setServoJointPulse(R_RTWIST, joints["RTWIST"]);
      robot.setServoJointPulse(R_RKNEE, joints["RKNEE"]);
      robot.setServoJointPulse(R_LARM, joints["LARM"]);
      robot.setServoJointPulse(R_RARM, joints["RARM"]);
      robot.setServoJointPulse(R_EYES, joints["EYES"]);
      robot.setServoJointPulse(R_AUX1, joints["AUX1"]);
      robot.setServoJointPulse(R_AUX2, joints["AUX2"]);
    } else { robot.stopRobot(); }

    if (!saving) {
      if ((c = getch())) {
        if (c == 'q') { joints["LHIP"]++; }
        if (c == 'a') { joints["LHIP"]--; }
        if (c == 'w') { joints["LTWIST"]++; }
        if (c == 's') { joints["LTWIST"]--; }
        if (c == 'e') { joints["LKNEE"]++; }
        if (c == 'd') { joints["LKNEE"]--; }
        if (c == 'r') { joints["RHIP"]++; }
        if (c == 'f') { joints["RHIP"]--; }
        if (c == 't') { joints["RTWIST"]++; }
        if (c == 'g') { joints["RTWIST"]--; }
        if (c == 'y') { joints["RKNEE"]++; }
        if (c == 'h') { joints["RKNEE"]--; }
        if (c == 'z') { joints["LARM"]++; }
        if (c == 'x') { joints["LARM"]--; }
        if (c == 'c') { joints["RARM"]++; }
        if (c == 'v') { joints["RARM"]--; }
        if (c == 'u') { joints["EYES"]++; }
        if (c == 'j') { joints["EYES"]--; }
        if (c == 'i') { joints["AUX1"]++; }
        if (c == 'k') { joints["AUX1"]--; }
        if (c == 'o') { joints["AUX2"]++; }
        if (c == 'l') { joints["AUX2"]--; }
        if (c == 'n') { disabled = !disabled; }
        if (c == '\n') { saving = true; }
      }
    } else {
      INFO("Are you sure you wish to save these values? (y/n) ");
      c = getchar();
      if (c == 'y') { INFO("Saved!\n"); break; } else { saving = false; }
    }
    //setServoPulse(i2cptr, LANKLEADD, curval);
    usleep(10000);
  }

  if (saving) { writeCalVals(joints); }
  return 1;
}

float gettime() {
  struct timespec monotime;
  clock_gettime(CLOCK_MONOTONIC, &monotime);
  return (monotime.tv_sec + ((float)monotime.tv_nsec) / 1000000000);
}

void writeCalVals(map<string, int> joints) {
  std::ofstream of ("../shared/martyJointCalib.h");
  of << "#ifndef __ARCHIE_JOINT_CALIB" << endl;
  of << "#define __ARCHIE_JOINT_CALIB" << endl;
  of << "#define CALIBRATED\ttrue" << endl << endl;
  int offset(0);
  for (auto i = joints.begin(); i != joints.end(); i++) {
    string name = i->first; int zero = i->second;
    if ((name == "LHIP") or (name == "RHIP")) {offset = HIPOFFSET;}
    else if ((name == "LTWIST") or (name == "RTWIST")) {offset = TWISTOFFSET;}
    else if ((name == "LKNEE") or (name == "RKNEE")) {offset = KNEEOFFSET;}
    else if ((name == "LARM") or (name == "RARM")) {offset = ARMOFFSET;}
    else if (name == "EYES") {offset = EYESOFFSET;}
    else if ((name == "AUX1") or (name == "AUX2")) {offset = AUXOFFSET;}
    else {WARN("No offset found for joint: " << name << endl);}
    // DEBUG(name << " " << zero << endl);
    of << "#define " << name << "ZERO\t" << zero << endl;           // ZeroPos
    of << "#define " << name << "MAX\t" << (zero + offset) << endl;  // MaxPos
    of << "#define " << name << "MIN\t" << (zero - offset) << endl;  // MinPos
    of << endl;
  }
  of << "#endif" << endl;
  of.close();
}

char getch() {
  char buf = 0;
  struct termios old = {0};
  if (tcgetattr(0, &old) < 0)
    perror("tcsetattr()");
  old.c_lflag &= ~ICANON;
  old.c_lflag &= ~ECHO;
  old.c_cc[VMIN] = 1;
  old.c_cc[VTIME] = 0;
  if (tcsetattr(0, TCSANOW, &old) < 0)
    perror("tcsetattr ICANON");
  if (read(0, &buf, 1) < 0)
    perror ("read()");
  old.c_lflag |= ICANON;
  old.c_lflag |= ECHO;
  if (tcsetattr(0, TCSADRAIN, &old) < 0)
    perror ("tcsetattr ~ICANON");
  return (buf);
}
