#ifndef PARAMLOADER_H
#define PARAMLOADER_H
//-----------------------------------------------------------------------------
//
//  Name:   ParamLoader.h
//
//  Example Code from the book "Programming Game AI by Example", Author: Mat Buckland 2002 (fup@ai-junkie.com)
//	Modified 27 feb 07 by Brett Jones

//  Desc:   class to setup parameters for the steering behavior project
//-----------------------------------------------------------------------------

// TODO: Rework or lose this class - either read from file again or use gui sliders to modify values
#include "misc/utils.h"

#define Prm (*ParamLoader::Instance())

class ParamLoader
{
private:
  
	//constructor
	ParamLoader()
	{
    //probably wont be used
    NumObstacles            = 1;
    MinObstacleRadius       = 50;
    MaxObstacleRadius       = 60;

    NumSamplesForSmoothing  = 10;

    SteeringForceTweaker    = 50.0;
    MaxSteeringForce        = 2 * SteeringForceTweaker;
    MaxSpeed                = 70;
    VehicleMass             = 1;
    VehicleScale            = 0.5;

    SeparationWeight        = 3 * SteeringForceTweaker;
    AlignmentWeight         = 1 * SteeringForceTweaker;
    CohesionWeight          = 2 * SteeringForceTweaker;
    ObstacleAvoidanceWeight = 10 * SteeringForceTweaker;
    WallAvoidanceWeight     = 2 * SteeringForceTweaker;
    WanderWeight            = 0.25 * SteeringForceTweaker;
    SeekWeight              = 1 * SteeringForceTweaker;
    FleeWeight              = 1 * SteeringForceTweaker;
    ArriveWeight            = 1 * SteeringForceTweaker;
    PursuitWeight           = 1 * SteeringForceTweaker;
    OffsetPursuitWeight     = 1 * SteeringForceTweaker;
    InterposeWeight         = 1 * SteeringForceTweaker;
    HideWeight              = 1 * SteeringForceTweaker;
    EvadeWeight             = 1 * SteeringForceTweaker;
    FollowPathWeight        = 5 * SteeringForceTweaker;

    ViewDistance            = 150;
    MinDetectionBoxLength   = 50;
    WallDetectionFeelerLength = 50;

    prWallAvoidance         = 0.5;
    prObstacleAvoidance     = 0.5;  
    prSeparation            = 0.2;
    prAlignment             = 0.3;
    prCohesion              = 0.6;
    prWander                = 0.8;
    prSeek                  = 0.8;
    prFlee                  = 0.6;
    prEvade                 = 1.0;
    prHide                  = 0.8;
    prArrive                = 0.5;

    //MaxTurnRatePerSecond    = Pi;
	//MaxTurnRatePerSecond  =  2*(Pi/360); //  1 degree
	MaxTurnRatePerSecond  =  (Pi/360); //      0.5 degree


	}

public:

  static ParamLoader* Instance();

  //probably wont be used
  int	 NumObstacles;
  double MinObstacleRadius;
  double MaxObstacleRadius;

  //how many samples the smoother will use to average a value
  int   NumSamplesForSmoothing;

  //used to tweak the combined steering force (simply altering the MaxSteeringForce
  //will NOT work!This tweaker affects all the steering force multipliers too).
  double SteeringForceTweaker;
  double MaxSteeringForce;
  double MaxSpeed;
  double VehicleMass;

  double VehicleScale;
  double MaxTurnRatePerSecond;

  double SeparationWeight;
  double AlignmentWeight ;
  double CohesionWeight  ;
  double ObstacleAvoidanceWeight;
  double WallAvoidanceWeight;
  double WanderWeight    ;
  double SeekWeight      ;
  double FleeWeight      ;
  double ArriveWeight    ;
  double PursuitWeight   ;
  double OffsetPursuitWeight;
  double InterposeWeight ;
  double HideWeight      ;
  double EvadeWeight     ;
  double FollowPathWeight;
 
  //how close a neighbour must be before an agent perceives it (considers it
  //to be within its neighborhood)
  double ViewDistance;

   //used in obstacle avoidance
  double MinDetectionBoxLength;

  //used in wall avoidance
  double WallDetectionFeelerLength;

  //these are the probabilities that a steering behavior will be used
  //when the prioritized dither calculate method is used
  double prWallAvoidance;
  double prObstacleAvoidance;
  double prSeparation;
  double prAlignment;
  double prCohesion;
  double prWander;
  double prSeek;
  double prFlee;
  double prEvade;
  double prHide;
  double prArrive;
  
};

#endif
