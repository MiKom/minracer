/***************************************************************************

    file                 : minracer.cpp
    created              : Sat Jan 9 17:56:45 CET 2010
    copyright            : (C) 2010 MasterM and MiKom

 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>
#include "ffll/FFLLAPI.h"
#include "model.h"

#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

static tTrack	*curTrack;
static Model    *modSteer;
static Model    *modAccel;
static Model    *modGears;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 


/* 
 * Module entry point  
 */ 
extern "C" int 
minracer(tModInfo *modInfo) 
{
    memset(modInfo, 0, 10*sizeof(tModInfo));

    modInfo->name    = "minracer";		/* name of the module (short) */
    modInfo->desc    = "";	/* description of the module (can be long) */
    modInfo->fctInit = InitFuncPt;		/* init function */
    modInfo->gfId    = ROB_IDENT;		/* supported framework version */
    modInfo->index   = 1;

    return 0; 
} 

/* Module interface initialization. */
static int 
InitFuncPt(int index, void *pt) 
{ 
    tRobotItf *itf  = (tRobotItf *)pt; 

    itf->rbNewTrack = initTrack; /* Give the robot the track view called */ 
				 /* for every track change or new race */ 
    itf->rbNewRace  = newrace; 	 /* Start a new race */
    itf->rbDrive    = drive;	 /* Drive during race */
    itf->rbPitCmd   = NULL;
    itf->rbEndRace  = endrace;	 /* End of the current race */
    itf->rbShutdown = shutdown;	 /* Called before the module is unloaded */
    itf->index      = index; 	 /* Index used if multiple interfaces */

    Model::set_base(".torcs/drivers/minracer/rules");
    return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
    curTrack = track;
    *carParmHandle = NULL; 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{
  modSteer = new Model("steer.fcl");
  modAccel = new Model("accel.fcl");
  modGears = new Model("gears.fcl");
}

#define MAXDISTANCE 200.0f
#define MAXRADIUS 200.0f
float util_get_nturn(tCarElt* car, const float maxdist, const float maxrad, float* radius)
{
  float dist = 0.0f;
  tTrackSeg *seg = car->_trkPos.seg;

  if(seg->type == TR_STR)
    dist -= car->_trkPos.toStart;
  
  while(seg->type == TR_STR && dist < maxdist) {
    dist += seg->length;
    seg   = seg->next;
  }

  if(dist > maxdist) {
    *radius = maxrad;
    dist = maxdist;
  }
  else {
    *radius = seg->radius;
    if(*radius > maxrad) *radius = maxrad;
  }
  return dist;
}

float util_get_speed(tCarElt* car)
{
  return sqrtf(
	       car->_speed_x*car->_speed_x +
	       car->_speed_y*car->_speed_y +
	       car->_speed_z*car->_speed_z) * 3.6f;
}

float util_get_maxdist(const float speed, const float minv, const float threshold)
{
  float maxdist = speed - threshold;
  if(maxdist < minv)
    return minv;
  else return maxdist * 1.5f;
}

#define ACCEL_SPEED    0
#define ACCEL_DISTANCE 1
#define ACCEL_RADIUS   2
#define ACCEL_OUTPUT   3
static float drive_accel(tCarElt* car, tSituation *s)
{
  float distance, speed, radius, maxdist;
  speed    = util_get_speed(car);
  maxdist  = util_get_maxdist(speed, 20.0f, 100.0f);
  distance = util_get_nturn(car, maxdist, MAXRADIUS, &radius);
  
  modAccel->set(ACCEL_SPEED, speed + 1.0f);
  modAccel->set(ACCEL_DISTANCE, distance / maxdist);
  modAccel->set(ACCEL_RADIUS, radius);

  float accel = modAccel->compute();
  if(accel < 0.0f) {
    car->_brakeCmd = fabsf(accel);
    car->_accelCmd = 0.0f;
  }
  else {
    car->_accelCmd = fabsf(accel);
    car->_brakeCmd = 0.0f;
  }

  //printf("S %.2f D %.2f R %.2f DR %.2f MD %.2f ",
  //	 speed, distance, radius, distance/maxdist, maxdist);
  return accel;
}

#define STEER_TOMIDDLE 0
#define STEER_ANGLE    1
#define STEER_OUTPUT   2
static float drive_steer(tCarElt* car, tSituation *s)
{
  float angle, tm, steer;
  angle = RtTrackSideTgAngleL(&(car->_trkPos)) - car->_yaw;
  tm    = 2.0f * car->_trkPos.toMiddle / car->_trkPos.seg->width;
  NORM_PI_PI(angle);

  modSteer->set(STEER_ANGLE, angle);
  modSteer->set(STEER_TOMIDDLE, tm);
  steer = modSteer->compute();
  car->_steerCmd = steer;

  //  printf("A %.2f TM %.2f ", angle, tm);
  return steer;
}

#define GEARS_ACCEL  0
#define GEARS_RPM    1
#define GEARS_OUTPUT 2
static int drive_gears(tCarElt* car, const float accel)
{
  float gear_ratio = car->_enginerpm / car->_enginerpmRedLine;
  int gear_cmd;
  modGears->set(GEARS_ACCEL, accel);
  modGears->set(GEARS_RPM, gear_ratio);

  gear_cmd = (int)modGears->compute();
  if(car->_gear < 1) gear_cmd = 1;
  else if(car->_gear == car->_gearNb-1)
    gear_cmd = gear_cmd==1?0:gear_cmd;
  
  car->_gearCmd = car->_gear + gear_cmd;

  //printf("GR %.2f ", gear_ratio);
  return gear_cmd;
}

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{
  memset((void *)&car->ctrl, 0, sizeof(tCarCtrl));
  float steer = drive_steer(car, s);
  float accel = drive_accel(car, s);
  int gearcmd = drive_gears(car, accel);
  
  /*printf("-> %.2f %.2f %d     \r",
    steer, accel, gearcmd);
    fflush(stdout);*/
}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
  delete modSteer;
  delete modAccel;
  delete modGears;
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

