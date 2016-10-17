#ifndef __ARCHIE_UTILS
#define __ARCHIE_UTILS

#include <time.h>
#include <iostream>
#include <fstream>
#include <string.h>
#include <vector>
#include <map>
#include <unistd.h>
#include <termios.h>
#include "martyrobot.h"

using namespace std;



// Methods
int manualCalibration(martyrobot& robot);
char getch();
float gettime();
void writeCalVals(map<string, int> joints);

#endif
