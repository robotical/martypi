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
  INFO("CALIBRATE: Starting\n");
  martyrobot robot;
  robot.init(true);

  manualCalibration(robot);

  return 1;
}
