#ifndef OBSTACLE_H
#define OBSTACLE_H

//------------------------------------------------------------------------
//
//  Name:   Obstacle.h
//
//  Desc:   Simple obstacle class
//
//  Author: Mat Buckland 2002 (fup@ai-junkie.com)
//  Last Modifed 27 feb 07 by Brett Jones
//
//------------------------------------------------------------------------

#include "BaseGameEntity.h"
#include "AIWorld.h"
#include <windows.h>

using namespace Ogre;

//
class AIWorld;

class Obstacle : public BaseGameEntity
{
private:

  //at the moment an obstacle is stationary - in future should be dynamic if desired
	SceneNode*  ObstacleNode;
	Entity*     ObstacleEntity;

	AIWorld*    m_pWorld;

public:

  //ctor
  Obstacle(AIWorld* m_pWorld, Vector3 position);

  //dtor
  virtual ~Obstacle();

  void LoadMesh();
  void removeMeshEntity();

  Ogre::SceneNode* getNode() { return ObstacleNode; }

};

#endif

