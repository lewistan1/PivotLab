#ifndef FRAMES_H
#define FRAMES_H

#include <Arduino.h>

#include "frame_00_delay_0_07s.h"
#include "frame_01_delay_0_07s.h"
#include "frame_02_delay_0_07s.h"
#include "frame_03_delay_0_07s.h"
#include "frame_04_delay_0_07s.h"
#include "frame_05_delay_0_07s.h"
#include "frame_06_delay_0_07s.h"
#include "frame_07_delay_0_07s.h"
#include "frame_08_delay_0_07s.h"
#include "frame_09_delay_0_07s.h"
#include "frame_10_delay_0_07s.h"
#include "frame_11_delay_0_07s.h"
#include "frame_12_delay_0_07s.h"
#include "frame_13_delay_0_07s.h"
#include "frame_14_delay_0_07s.h"
#include "frame_15_delay_0_07s.h"
#include "frame_16_delay_0_07s.h"
#include "frame_17_delay_0_07s.h"
#include "frame_18_delay_0_07s.h"
#include "frame_19_delay_0_07s.h"
#include "frame_20_delay_0_07s.h"
#include "frame_21_delay_0_07s.h"
#include "frame_22_delay_0_07s.h"
#include "frame_23_delay_0_07s.h"
#include "frame_24_delay_0_07s.h"
#include "frame_25_delay_0_07s.h"

const unsigned char* frames[] PROGMEM = {
  frame_00_delay_0_07s, frame_01_delay_0_07s, frame_02_delay_0_07s, frame_03_delay_0_07s, frame_04_delay_0_07s, 
  frame_05_delay_0_07s, frame_06_delay_0_07s, frame_07_delay_0_07s, frame_08_delay_0_07s, frame_09_delay_0_07s, 
  frame_10_delay_0_07s, frame_11_delay_0_07s, frame_12_delay_0_07s, frame_13_delay_0_07s, frame_14_delay_0_07s, 
  frame_15_delay_0_07s, frame_16_delay_0_07s, frame_17_delay_0_07s, frame_18_delay_0_07s, frame_19_delay_0_07s, 
  frame_20_delay_0_07s, frame_21_delay_0_07s, frame_22_delay_0_07s, frame_23_delay_0_07s, frame_24_delay_0_07s, 
  frame_25_delay_0_07s
};

const int FRAME_COUNT = sizeof(frames) / sizeof(frames[0]);

#endif
