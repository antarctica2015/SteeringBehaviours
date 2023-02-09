#ifndef AIWorld_H
#define AIWorld_H
#pragma warning (disable:4786)
#pragma once
//------------------------------------------------------------------------
//
//  Name:   GameWorld.h
//
//  Desc:   All the environment data and methods for the Steering Behavior projects. 
//
//  Example Code from the book "Programming Game AI by Example", Author: Mat Buckland 2002 (fup@ai-junkie.com)
//	Modified 17 April 06 by Brett Jones
//------------------------------------------------------------------------

#include <vector>

//TODO: remove  dependancy on this
#include "time/PrecisionTimer.h"

#include "EntityFunctionTemplates.h"
#include "Obstacle.h"
#include "vehicle.h"
#include "SteeringBehaviors.h"


//should either be read from file? or changable from GUI
//this is for debug/testing only!
#define NUMAGENTS               1
#define STEERINGFORCETWEAKER    100
#define NUMSAMPLESFORSMOOTHING  10
#define MAXFORCE				2.0*STEERINGFORCETWEAKER
#define MAXSPEED				100.0
#define MAXTURNRATE			   (2*(Pi/360)) //1 degree
#define MASS				    1.0
#define SCALE					0.5

using namespace Ogre;

//forward dec
class State;
class Path;
class Obstacle;
 
//list all the states
enum demo_state_type
  {
    none,
    seek,
    flee,
    arrive,
	evade,
	pursuit,
	offset_pursuit,
    interpose,
    flock,
	follow_path,
	wander,
	obstacle_avoidance
  };

class AIWorld
{ 
private:

  SceneNode* boundsNode;
  Entity*    boxEnt;
  
  //pointer to scenemanager singleton for access to all nodes/entities and geometry 
  SceneManager* AISceneMgr; 

  //a container of all the moving entities
  std::vector<AIVehicle*>      m_Vehicles;

   //any obstacles
  std::vector<BaseGameEntity*>  m_Obstacles;

  //any path we may create for the vehicles to follow
  Path*                         m_pPath;

  //the position of the crosshair/target point
  Vector3                      m_vCrosshair;

  //keeps track of the average FPS
  double                       m_dAvFrameTime;

  //current state
  State*                       m_pCurrentState;

public:
  
  //constructor
  AIWorld(SceneManager* aiSceneMgr); 

  //deconstructor
  ~AIWorld();

  //TODO: rework this to be user friendly/use a line drawing class
  void showWorldBounds();

  //set up some obstacles
  void CreateObstacles();

   void RemoveObstacles();

  // add a new vehicle to the world
  void AddVehicle();

  //TODO: remove a specific vehicle from the world by id
  void RemoveVehicle();

  //remove all vehicles from the world
  void RemoveAllVehicles();

  void  Update(double time_elapsed);  
  
  //enum for state selection via menu
  demo_state_type              current_state;
  
  //gets a new state from menu
  void SelectState(demo_state_type e_state); 
  
  //changes to a new state
  void ChangeState(State* new_state); 

 //enforce seperation- eventually will be replaced with better collision detection & response via OgreNewt
 void  NonPenetrationContraint(AIVehicle* v){EnforceNonPenetrationConstraint(v, m_Vehicles);}

 //identify neighbouring vehicles within a certain range: used to work out steering force in flocking
 //replace with bounds checking via OgreNewt
 void  TagVehiclesWithinViewRange(BaseGameEntity* pVehicle, double range)
  {
   TagNeighbors(pVehicle, m_Vehicles, range);
  }

 // replace with bounds checks via Ogre or OgreNewt
  void  TagObstaclesWithinViewRange(BaseGameEntity* pVehicle, double range)
  {
    TagNeighbors(pVehicle, m_Obstacles, range);
  }


 //accessor method for Vehicle container
 const std::vector<AIVehicle*>& Agents(){return m_Vehicles;}
 
 //accessor method for Obstacle container
 const std::vector<BaseGameEntity*>& Obstacles()const{return m_Obstacles;}

 //get 'n' set for target position (used for various behaviours)
 Vector3     Crosshair()const{return m_vCrosshair;}
 void        SetCrosshair(Vector3 v){m_vCrosshair=v;}

 //return a pointer to the main scene manager
 Ogre::SceneManager* getSceneMgr() { return AISceneMgr; }
 
};
#endif