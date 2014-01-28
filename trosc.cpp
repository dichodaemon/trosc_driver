/***************************************************************************

    file                 : trosc.cpp
    created              : Wed Dec 11 16:54:59 CET 2013
    copyright            : (C) 2002 Dizan Vasquez

 ***************************************************************************/

#ifdef _WIN32
#include <windows.h>
#endif

#include <stdio.h>
#include <stdlib.h> 
#include <string.h> 
#include <math.h>

//#include <tgf.h> 
#include <track.h> 
#include <car.h> 
#include <raceman.h> 
#include <robottools.h>
#include <robot.h>

#include "facade.h"
#include "DataCollection.h"
#include <algorithm>
#include <iostream>

static tTrack	*curTrack;

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
trosc(tModInfo *modInfo) 
{
  memset(modInfo, 0, 10*sizeof(tModInfo));

  modInfo->name    = strdup("trosc");		/* name of the module (short) */
  modInfo->desc    = strdup("");	/* description of the module (can be long) */
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
  return 0; 
} 

/* Called for every track change or new race. */
static void  
initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s) 
{ 
  curTrack = track;
  *carParmHandle = NULL;

  InitTrackData(track); 
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{
  InitCarData(car);
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 

  memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
  Command c = GetCommandData();
  if ( abs( c.steering ) > car->_steerLock ) {
    if ( c.steering > 0.0 ) {
      c.steering = c.steering * car->_steerLock;
    } else {
      c.steering = -c.steering * car->_steerLock;
    }
  }
  c.steering /= car->_steerLock;
  car->ctrl.steer = c.steering;
  car->ctrl.brakeCmd = c.brake;
  car->ctrl.accelCmd = c.acceleration;
  car->ctrl.gear = c.gear;


/*  car->ctrl.steer = 0;
  car->ctrl.brakeCmd = 0;
  car->ctrl.accelCmd = 0.1;
  car->ctrl.gear = 1;
*/
  SendMessages(index, car, s);

}

/* End of the current race */
static void
endrace(int index, tCarElt *car, tSituation *s)
{
}

/* Called before the module is unloaded */
static void
shutdown(int index)
{
}

