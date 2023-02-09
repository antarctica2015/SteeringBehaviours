#include "Obstacle.h"

using namespace Ogre;

Obstacle::Obstacle(AIWorld* aiWorld, Vector3 vecpos):BaseGameEntity(0, vecpos)
{
	m_pWorld = aiWorld;
	SetPosition(vecpos);

	LoadMesh();
}

Obstacle::~Obstacle()
{
	
}

void Obstacle::LoadMesh()
{
	const Real scale = 0.25; 
	Vector3 vScale(scale,scale,scale);  // mesh scale
	
	//select which mesh to use and what its name is (from individual id)
	ObstacleEntity = m_pWorld->getSceneMgr()->createEntity("obstacle" + StringConverter::toString(ID()),"geosphere4500.mesh" );
	ObstacleEntity->setMaterialName("Simple/Translucent");

	// so lighting works
	ObstacleEntity->setNormaliseNormals(true);

	//attatch node as a subnode of root
	ObstacleNode = m_pWorld->getSceneMgr()->getRootSceneNode()->createChildSceneNode();
	mNode = m_pWorld->getSceneMgr()->getRootSceneNode()->createChildSceneNode();
	
	//attatch the mesh to unitnode
	ObstacleNode->attachObject( ObstacleEntity );
	
	//set node (and hence mesh)scale
	ObstacleNode->setScale(vScale);

	ObstacleNode->setVisible(true);

	//set position in world (sphere mesh node)
	ObstacleNode->setPosition( GetPosition() );

	//circle node
	mNode->setPosition( GetPosition() );

	//mNode->addChild(ObstacleNode);

	//get mesh radius - use to create a bounding sphere
	SetRadius( ObstacleEntity->getBoundingRadius() );
}

void Obstacle::removeMeshEntity()
{
	ObstacleNode->setVisible(false);
	ObstacleNode->detachAllObjects();

	Ogre::String entName = ObstacleEntity->getName();
	m_pWorld->getSceneMgr()->destroyEntity(entName);
	
	Ogre::String nodeName = ObstacleNode->getName();
	m_pWorld->getSceneMgr()->destroySceneNode(nodeName);

	mNode->setVisible(false);
	mNode->detachAllObjects();

	Ogre::String mName = mNode->getName();
	m_pWorld->getSceneMgr()->destroySceneNode(mName);

}