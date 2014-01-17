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

#include <algorithm>
#include <iostream>

static tTrack	*curTrack;

static void initTrack(int index, tTrack* track, void *carHandle, void **carParmHandle, tSituation *s); 
static void newrace(int index, tCarElt* car, tSituation *s); 
static void drive(int index, tCarElt* car, tSituation *s); 
static void endrace(int index, tCarElt *car, tSituation *s);
static void shutdown(int index);
static int  InitFuncPt(int index, void *pt); 

#define CURVATURE_TOLERANCE 0.5
#define CURVATURE_MAX_DISTANCE 200

Facade facade( 6000 );

float computeCurvature( tTrackSeg * segment ) 
{
  if ( segment->type == TR_STR ) {
    return 0;
  } else if( segment->type == TR_LFT ) {
    return 1.0 / segment->radius;
  } else {
    return -1.0 / segment->radius;
  }
}

void nextCurve( tCarElt * car, Status & status )
{
  float curvature = computeCurvature( car->_trkPos.seg );;
  float cummulated = 0;
  if ( car->_trkPos.seg->type == TR_STR ) {
    cummulated = car->_trkPos.seg->length - car->_trkPos.toStart;
  } else {
    cummulated = ( car->_trkPos.seg->arc - car->_trkPos.toStart ) * car->_trkPos.seg->radius;
  }


  tTrackSeg * current = car->_trkPos.seg->next;
  
  while ( cummulated < CURVATURE_MAX_DISTANCE ) {
    float currentCurvature = computeCurvature( current );
    float ratio = 1.0;
    if ( currentCurvature != 0.0 ) {
      ratio = abs( curvature / currentCurvature );
    } else if ( curvature != currentCurvature ) {
      ratio = 1.0 + 10 * CURVATURE_TOLERANCE;
    }

    if ( ratio > 1.0 + CURVATURE_TOLERANCE || ratio < 1.0 - CURVATURE_TOLERANCE ) {
      status.nextCurvature = currentCurvature;
      status.nextDistance = cummulated;
      return;
    }
    cummulated += current->length;
    current = current->next;
  }
  status.nextCurvature = curvature;
  status.nextDistance = cummulated;
}

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
} 

/* Start a new race. */
static void  
newrace(int index, tCarElt* car, tSituation *s) 
{ 
} 

/* Drive during race. */
static void  
drive(int index, tCarElt* car, tSituation *s) 
{ 
  memset((void *)&car->ctrl, 0, sizeof(tCarCtrl)); 
  Command c = facade.getCommand();
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

  Status status;
  status.rpm  = car->priv.enginerpm;
  status.gear = car->priv.gear;
  status.gearRatio = car->_gearRatio[car->_gear + car->_gearOffset];
  status.lowerGearRatio = car->_gearRatio[car->_gear + car->_gearOffset - 1];
  status.maxRPM = car->_enginerpmMax;
  status.wheelRadius = car->_wheelRadius( REAR_RGT );

  
  float yaw = car->_yaw - RtTrackSideTgAngleL( &car->_trkPos );
  NORM_PI_PI( yaw );
  status.trackYaw = yaw;
  status.trackDistance = car->_trkPos.toMiddle;
  status.trackCurvature = computeCurvature( car->_trkPos.seg );
  status.trackWidth = car->_trkPos.seg->width;
  nextCurve( car, status );

  status.speed = car->_speed_x;
  status.yaw = car->_yaw;
  status.x = car->_pos_X;
  status.y = car->_pos_Y;

  facade.setStatus( status );

  Obstacles obstacles( s->_ncars - 1 );
  int count = 0;
  for ( int i = 0; i <= obstacles.size(); ++i ) {
    if ( s->cars[i]->index != car->index ) {
      float x = s->cars[i]->_pos_X - status.x;
      float y = s->cars[i]->_pos_Y - status.y;
      float yaw = s->cars[i]->_yaw - status.yaw;
      NORM_PI_PI( yaw );
      Obstacle & o = obstacles[count];
      o.id = s->cars[i]->index;
      o.x = x * cos( -status.yaw ) - y * sin( -status.yaw );
      o.y = y * cos( -status.yaw ) + x * sin( -status.yaw );
      o.theta = yaw;
      o.vX = s->cars[i]->_speed_x * cos( yaw ) + s->cars[i]->_speed_y * cos( yaw + 3.14159265 );
      o.vY = s->cars[i]->_speed_x * sin( yaw ) + s->cars[i]->_speed_y * sin( yaw + 3.14159265 );
      o.width = s->cars[i]->_dimension_y;
      o.height = s->cars[i]->_dimension_x;
      count++;
    }
  }

  facade.setObstacles( obstacles );
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

