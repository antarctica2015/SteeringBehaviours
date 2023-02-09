#ifndef BASE_GAME_ENTITY_H
#define BASE_GAME_ENTITY_H
#pragma once

#include <OgreNewt.h>
#include <vector>
#include <string>
#include "misc/Utils.h"

using namespace Ogre;

class BaseGameEntity
{
public:
	enum {default_entity_type = -1};

//private: //TODO: should be private - not atm for convienience

  //each entity has a unique ID
  int         m_ID;

  //every entity has a type associated with it (health, troll, ammo etc)
  //could do with explictly setting up enum/entity types
  int         m_EntityType;

  //this is a generic flag. 
  bool        m_bTag;

  // position vector
  Vector3    m_vPosition;

   // vehicle bounding radius 
  Real        m_Radius;

   //used by the constructor to give each entity a unique ID
  int NextValidID(){static int NextID = 0; return NextID++;}

public:

  BaseGameEntity(int entity_type, Vector3 mPosition):m_vPosition(mPosition),
                                        m_ID(NextValidID()),
                                        m_EntityType(entity_type),
										m_bTag(false) {}

  virtual ~BaseGameEntity() {}

  virtual void removeMeshEntity() {};

  SceneNode*  mNode;

  Vector3      GetPosition()const{return m_vPosition;}
  void         SetPosition(Vector3 new_pos){m_vPosition = new_pos;}

  Real         GetRadius()const{return m_Radius;}
  void         SetRadius(double r){m_Radius = r;}
  int          ID()const{return m_ID;}

  bool         IsTagged()const{return m_bTag;}
  void         Tag(){m_bTag = true;}
  void         UnTag(){m_bTag = false;}

  int       GetEntityType()const{return m_EntityType;}
  void      SetEntityType(int new_type){m_EntityType = new_type;}

   Ogre::SceneNode* getSceneNode() { return mNode; }

};

      
#endif