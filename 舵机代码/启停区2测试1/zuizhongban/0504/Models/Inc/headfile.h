#ifndef _HEADFILE_H
#define _HEADFILE_H

#define EMM_V5_ENABLE  
// #define ZDT_X42_V2_ENABLE  

#include "lcd_fonts.h"
#include "lcd_spi_169.h"
#include "lcd_image.h"

#include "imu.h"
#include "camera.h"

#ifdef EMM_V5_ENABLE
#include "Emm_V5.h"
#endif

#ifdef ZDT_X42_V2_ENABLE
#include "ZDT_X42_V2.h"
#endif

#include "move.h"
#include "servo.h"
#include "pid.h"
#include "task.h"
#include "hmi.h"
#include "bfs.h"

#include <math.h>

#endif
