

#include "AIWorld.h"
#include "Vehicle.h"
#include "DemoStates.h"
#include "misc/utils.h"
#include <list>
using std::list;
using namespace Ogre;

//------------------------------- ctor -----------------------------------
//------------------------------------------------------------------------
AIWorld::AIWorld(SceneManager* aiSceneMgr): m_vCrosshair(Vector3(0.0,0.0,0.0)),
											m_dAvFrameTime(0.0),
											m_pCurrentState(IdleDemo::Instance()),
											AISceneMgr(aiSceneMgr),
											current_state(none)
{
	
	//NOTE- this is extremely hacky and very temp
	//cheap way to show world bounds - to be replaced
	boxEnt = AISceneMgr->createEntity("boxentity" ,"box.mesh" );
	boxEnt->setMaterialName("Simple/Translucent");

	//attatch node as a subnode of root
	boundsNode = AISceneMgr->getRootSceneNode()->createChildSceneNode();
	
	//attatch the mesh to unitnode
	boundsNode->attachObject( boxEnt );

	//show node bounds
	boundsNode->setVisible(false);

	//set node (and hence mesh)scale
	boundsNode->setScale(2000,2000,2000);

	// so lighting works
	boxEnt->setNormaliseNormals(true);

	//show the AABB for the node
	boundsNode->showBoundingBox(true);

	// make a simple visual object for the crosshair plane. (at y = 0)
	Ogre::Entity* theplane = AISceneMgr->createEntity("ThePlane", Ogre::SceneManager::PT_PLANE );
	Ogre::SceneNode* planeNode = AISceneMgr->getRootSceneNode()->createChildSceneNode();

	theplane->setMaterialName( "Simple/Translucent" );
	planeNode->attachObject( theplane );
	planeNode->setPosition( Ogre::Vector3(0,0,0) );
	planeNode->setOrientation( Ogre::Quaternion( Ogre::Degree(-90), Ogre::Vector3(1,0,0) ) );

	planeNode->setVisible(false);
	planeNode->scale(10,10,10);
	planeNode->showBoundingBox(true);
}


//-------------------------------- dtor ----------------------------------
//------------------------------------------------------------------------
AIWorld::~AIWorld()
{
  for (unsigned int a=0; a<m_Vehicles.size(); ++a)
  {
    delete m_Vehicles[a];
  }

  //
  for (unsigned int ob=0; ob<m_Obstacles.size(); ++ob)
  {
	m_Obstacles[ob]->removeMeshEntity();
    delete m_Obstacles[ob];
  }

   //delete m_pPath;

}

void AIWorld::AddVehicle()
{
	
	//set a semi random start position in the range +/- 400 across XYZ
	Vector3 RndSpawnPos = Vector3(RandomClamped() * 900.0,
								  0.0, //RandomClamped() * 350
								  RandomClamped() * 900.0);

	AIVehicle* pVehicle = new AIVehicle(0,						  //entity type
										AISceneMgr,				  //scene manager
										this,					  //ai world
										RndSpawnPos,			  //initial position
										0,		                  //initial rotation
										Vector3(0.0,0.0,0.0),	  //initial velocity
										MAXFORCE,                 //max force
										MAXSPEED,				  //max speed
										MAXTURNRATE,		      //max turn rate
										MASS,					  //mass
										SCALE);                   //scale
	
	//select summing method
	pVehicle->Steering()->SetSummingMethod(SteeringBehavior::prioritized);
	//pVehicle->Steering()->SetSummingMethod(SteeringBehavior::weighted_average);
	
	//add to vector
    m_Vehicles.push_back(pVehicle);

}

void AIWorld::RemoveVehicle()
{
	//remove individual vehicle from world by id?, position?
	
}

void AIWorld::RemoveAllVehicles()
{
  
  for (unsigned int a=0; a<m_Vehicles.size(); ++a)
  {
	  m_Vehicles[a]->removeMeshEntity();
	  
	  //
	  m_Vehicles[a]->~AIVehicle();
  }

  //zero the vector
  m_Vehicles.clear();
}

//----------------------------- Update -----------------------------------
//------------------------------------------------------------------------
void AIWorld::Update(double time_elapsed)
{
 //draw a box showing world size
 //showWorldBounds();

  //update the vehicles
  for (unsigned int a=0; a<m_Vehicles.size(); ++a)
  {
    //
    m_Vehicles[a]->Update(time_elapsed);
  }

}

//--------------------------- SelectState -------------------------------------
// gets input from menu and uses it to switch states
// a proper state machine/manager is probably overkill in this case
// but should be considered if any more complexity is added
//-----------------------------------------------------------------------------
void AIWorld::SelectState(demo_state_type e_state)
{
	current_state = e_state;

  switch(current_state)
  {
  case none:

	  ChangeState(IdleDemo::Instance());
	  break;
  
  case seek:
	  
	  ChangeState(SeekDemo::Instance());
	  break;

  case flee:
	  
	  ChangeState(FleeDemo::Instance());
	  break;

  case arrive:
	  
	  ChangeState(ArriveDemo::Instance());
	  break;

  case evade:
	  
	  ChangeState(EvadeDemo::Instance());
	  break;

  case pursuit:
	  
	  ChangeState(PursuitDemo::Instance());
	  break;

  case offset_pursuit:
	  
	  ChangeState(OffsetPursuitDemo::Instance());
	  break;

  case interpose:
	  
	  ChangeState(InterposeDemo::Instance());
	  break;
    
  case flock:
	  
	  ChangeState(FlockDemo::Instance());
	  break;

  case follow_path:
	  
	  ChangeState(FollowPathDemo::Instance());
	  break;
 
  case wander:
	  
	  ChangeState(WanderDemo::Instance());
	  break;

  case obstacle_avoidance:
	  
	  ChangeState(ObstacleAvoidanceDemo::Instance());
	  break;

 default:
	 ChangeState(IdleDemo::Instance());
	 break;
  }
}

//--------------------------- ChangeState -------------------------------------
//-----------------------------------------------------------------------------
void AIWorld::ChangeState(State* pNewState)
{
  //make sure both states are both valid before attempting to 
  //call their methods
  assert (m_pCurrentState && pNewState);

  //call the exit method of the existing state
  m_pCurrentState->Exit(this);

  //change state to the new state
  m_pCurrentState = pNewState;

  //call the entry method of the new state
  m_pCurrentState->Enter(this);
}


void AIWorld::showWorldBounds()
{
 //do something 
}

//--------------------------- CreateObstacles -----------------------------
//
//  Sets up the vector of obstacles with random positions and sizes. Makes
//  sure the obstacles do not overlap
//------------------------------------------------------------------------
void AIWorld::CreateObstacles()
{
	//manually positioned for now for debug purposes

  	  Obstacle* ob1 = new Obstacle(this, Vector3(100.0,0.0,100.0) );
      m_Obstacles.push_back(ob1);

	  Obstacle* ob2 = new Obstacle(this, Vector3(100.0,0.0,-100.0) );
      m_Obstacles.push_back(ob2);

	  Obstacle* ob3 = new Obstacle(this, Vector3(-100.0,0.0,100.0) );
      m_Obstacles.push_back(ob3);

	  Obstacle* ob4 = new Obstacle(this, Vector3(-100.0,0.0,-100.0) );
      m_Obstacles.push_back(ob4);

	  Obstacle* ob5 = new Obstacle(this, Vector3(0.0,0.0,0.0) );
      m_Obstacles.push_back(ob5);
	  
	  Obstacle* ob6 = new Obstacle(this, Vector3(-250.0,0.0,0.0) );
      m_Obstacles.push_back(ob6);

	  Obstacle* ob7 = new Obstacle(this, Vector3(250.0,0.0,0.0) );
      m_Obstacles.push_back(ob7);

	  Obstacle* ob8 = new Obstacle(this, Vector3(0.0,0.0,-250.0) );
      m_Obstacles.push_back(ob8);

	  Obstacle* ob9 = new Obstacle(this, Vector3(0.0,0.0,250.0) );
      m_Obstacles.push_back(ob9);

	  Obstacle* ob10 = new Obstacle(this, Vector3(-300.0,0.0,-300.0) );
      m_Obstacles.push_back(ob10);

	  Obstacle* ob11 = new Obstacle(this, Vector3(300.0,0.0,300.0) );
      m_Obstacles.push_back(ob11);

	  Obstacle* ob12 = new Obstacle(this, Vector3(300.0,0.0,-300.0) );
      m_Obstacles.push_back(ob12);

	  Obstacle* ob13 = new Obstacle(this, Vector3(-300.0,0.0,300.0) );
      m_Obstacles.push_back(ob13);

	  Obstacle* ob14 = new Obstacle(this, Vector3(-300.0,0.0,150.0) );
      m_Obstacles.push_back(ob14);

	  Obstacle* ob15 = new Obstacle(this, Vector3(300.0,0.0,150.0) );
      m_Obstacles.push_back(ob15);

	  Obstacle* ob16 = new Obstacle(this, Vector3(-300.0,0.0,-150.0) );
      m_Obstacles.push_back(ob16);

	  Obstacle* ob17 = new Obstacle(this, Vector3(300.0,0.0,-150.0) );
      m_Obstacles.push_back(ob17);     
}

void AIWorld::RemoveObstacles()
{
  //
  for (unsigned int ob=0; ob<m_Obstacles.size(); ++ob)
  {
	m_Obstacles[ob]->removeMeshEntity();
    delete m_Obstacles[ob];
  }
 
  m_Obstacles.clear();
}