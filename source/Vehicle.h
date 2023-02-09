#ifndef AIVEHICLE_H
#define AIVEHICLE_H
#pragma warning (disable:4786)
#pragma once
//------------------------------------------------------------------------
//
//  Name:   Vehicle.h
//
//  Desc:   Definition of a simple vehicle that uses steering behaviors
//
//  Example Code from the book "Programming Game AI by Example", 
//  Author: Mat Buckland 2002 (fup@ai-junkie.com)
//	Last Modified 27 feb 07 by Brett Jones
//
//------------------------------------------------------------------------

#include <vector>
#include <list>
#include <string>
//#include <OgreNewt.h>

#include "BaseGameEntity.h"
#include "AIWorld.h"
#include "misc/Utils.h" // some handy stuff here

class AIWorld; // Replace with Ogre Gameworld/ scenemanager?
class SteeringBehavior;

using namespace Ogre;



///////////////////////////////////////////////////

class AIVehicle : public BaseGameEntity
{
//public:
  
//  enum {default_entity_type = -1};

private: 

  //ogre stuff
  Entity*	        AIEntity;
  SceneNode*        AIUnitNode;
  SceneManager*     AISceneMgr;

  SceneNode*        trailNode;
  RibbonTrail*      trail;
  NameValuePairList pairList;

  //Sphere* mSphere; need to write a new class to combine sphere & manual sphere mesh stuff
  //also need line and a OOBB/rectangle/cube too... sigh

  //a pointer to the world data. So a vehicle can access any obstacle, path, wall or agent data
  AIWorld*            m_pWorld;  // AIWord to OgreNewtWorld transformation here or similar
  // this should maybe be a pointer to the root scenenode? or just main scene manager?

  //the steering behavior class
  SteeringBehavior*     m_pSteering;

  //each entity has a unique ID
  //int         m_ID;

  //every entity has a type associated with it (health, troll, ammo etc)
  //could do with explictly setting up enum/entity types
  //int         m_EntityType;

  //this is a generic flag. 
  //bool        m_bTag;

  // position vector
  //Vector3    m_vPosition;

  //a normalized vector pointing in the direction the entity is heading. 
  Vector3    m_vHeading;

  // velocity vector 
  Vector3    m_vVelocity;

  //a vector perpendicular to the heading vector
  Vector3    m_vSide; 

  // implement physics via ogrenewt? - later!
  double     m_dMass;
  
  //the maximum speed this entity may travel at.
  double     m_dMaxSpeed;

  //the maximum force this entity can produce to power itself 
  //(think rockets and thrust)
  double     m_dMaxForce;
  
  //the maximum rate (radians per second)this vehicle can rotate         
  double     m_dMaxTurnRate;

  //keeps a track of the most recent update time. (some of the
  //steering behaviors make use of this - see Wander)
  double     m_dTimeElapsed;

  //eventually to be replaced with quaterions (kind of done)
  double     m_dRotation;

   // vehicle scale
  double     m_dScale;
 
  // vehicle bounding radius 
  //Real     m_Radius;

  //////////////////////////////////////////////////////////
 
  //disallow the copying of Vehicle types
  AIVehicle(const AIVehicle&);
  AIVehicle& operator=(const AIVehicle&);

public:

  AIVehicle(int entity_type,
				SceneManager* aiSceneMgr,
		        AIWorld*      aiWorld,
                Vector3       position,
                double        rotation,
                Vector3       velocity,
                double        max_force,
                double        max_speed,
                double        max_turn_rate,
                double        mass,
				double        scale);

  // deconstructor
  ~AIVehicle();

  //get n set......possibly info hiding not required here mind

  double    GetRotation()const{return m_dRotation;}
  void      SetRotation(double NewRot){m_dRotation = NewRot;}

  Vector3   GetHeading()const{return m_vHeading;}
  void      SetHeading(Vector3 new_heading);

  Vector3   GetVelocity()const{return m_vVelocity;}
  void      SetVelocity(const Vector3& NewVel){m_vVelocity = NewVel;}
  
  double    GetMaxForce()const{return m_dMaxForce;}
  void      SetMaxForce(double mf){m_dMaxForce = mf;}
  
  double    GetMaxSpeed()const{return m_dMaxSpeed;}                       
  void      SetMaxSpeed(double new_speed){m_dMaxSpeed = new_speed;}

  double    GetMaxTurnRate()const{return m_dMaxTurnRate;}
  void      SetMaxTurnRate(double val){m_dMaxTurnRate = val;}

  double    GetMass()const{return m_dMass;}
  void      SetMass(double new_mass){m_dMass = new_mass;}

  double    GetScale()const{return m_dScale;}
  void      SetScale(double new_scale){m_dScale = new_scale;}

  Vector3   GetSide()const{return m_vSide;}
  void      SetSide(){m_vSide = m_vHeading.perpendicular();} //unused atm

  // and the rest....

  Ogre::String DebugText;

  // set up & remove mesh
  void addMeshEntity(Vector3 position, double scale); 
  void removeMeshEntity();
 
  //set up & remove ribbon trail
  void addRibbonTrail();
  void removeRibbonTrail();
 
  //void CreateSphere(const std::string& strName, const float r, const int nRings = 16, const int nSegments = 16);

  bool      IsSpeedMaxedOut()const{return m_dMaxSpeed*m_dMaxSpeed >= (double)m_vVelocity.squaredLength();}
  double    Speed()const{return (double)m_vVelocity.length();}
  double    SpeedSq()const{return (double)m_vVelocity.squaredLength();}

  //return the scenenode this unit is attatched to
  Ogre::SceneNode* getNode() { return AIUnitNode; }
  
  //return a pointer to the scene manager
  Ogre::SceneManager* getSceneMgr() { return AISceneMgr; }
  	
  double    TimeElapsed()const{return m_dTimeElapsed;}

  //updates the vehicle's position and orientation
  void      Update(double time_elapsed);
                                                                
  //accessor methods for steering behaviours & world/manager class
  SteeringBehavior*const  Steering()const{return m_pSteering;}
  AIWorld*const           World()const{return m_pWorld;} 
  
};

#endif