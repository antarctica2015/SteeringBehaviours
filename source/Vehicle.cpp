#include "Vehicle.h"
#include "SteeringBehaviors.h"

using std::vector;
using std::list;
using namespace Ogre;

//----------------------------- ctor -------------------------------------
//------------------------------------------------------------------------
AIVehicle::AIVehicle(int entity_type,
					 Ogre::SceneManager *aiscenemgr,
					 AIWorld *aiworld,
					 Ogre::Vector3 position,
					 double rotation,
					 Ogre::Vector3 velocity,
					 double max_force,
					 double max_speed,
					 double max_turn_rate,
					 double mass,
					 double scale): BaseGameEntity(entity_type, position),
								    AISceneMgr(aiscenemgr),
									m_pWorld(aiworld),
									m_dTimeElapsed(0.0)
	
{
	//set up properties
    SetPosition(position);
	SetRotation(rotation);
	SetVelocity(velocity);
	SetMaxForce(max_force);
	SetMaxSpeed(max_speed);
	SetMaxTurnRate(max_turn_rate);
	SetMass(mass);
	SetScale(scale);

	//sphere stuff used for bounds checking
	//create sphere mesh for visual debugging
	//CreateSphere("Sphere" ,16);

	//ogre sphere for collsion detection
	//mSphere = new Sphere(position,m_Radius); //ogresphere

	//load the mesh
	addMeshEntity(m_vPosition,scale); //setup ogre node/entity & mesh
	
	//set up the steering behavior class
    m_pSteering = new SteeringBehavior(this);

}

//---------------------------- dtor -------------------------------------
//-----------------------------------------------------------------------
AIVehicle::~AIVehicle()
{
  // delete pointer to steering behaviours class
  delete m_pSteering;

}

//------------------------------ Update ----------------------------------
//
//  Updates the vehicle's position from a series of steering behaviors
//------------------------------------------------------------------------
void AIVehicle::Update(double time_elapsed)
{    
  //update the time elapsed
  m_dTimeElapsed = time_elapsed;

  //Vector3 doesnt automatically set a value in its constructor so better give it a default value
  Vector3 SteeringForce = Vector3::ZERO;

  //calculate the combined force from each steering behavior in the vehicle's list
  SteeringForce = m_pSteering->Calculate();
    
  //Acceleration = Force/Mass
  Vector3 acceleration = ( SteeringForce / m_dMass );

  //update velocity
  m_vVelocity += (acceleration * m_dTimeElapsed); 

  if( (double)m_vVelocity.length() > m_dMaxSpeed )
  {
	  m_vVelocity.normalise();

	  m_vVelocity *= m_dMaxSpeed;
  }

  //update the position
  m_vPosition += ((m_vVelocity) * time_elapsed);

  AIUnitNode->setPosition( m_vPosition );

  //TODO: replace with something less rubbish/hideous
  //quick n nasty way to keep units on screen
  //treat it as a toroid..
  //need to remove then reapply ribbon trails first
  
  if (m_vPosition.x > 1000.0)
  { 
	 removeRibbonTrail();
	 m_vPosition.x = -1000.0;
	 AIUnitNode->setPosition( m_vPosition );
	 addRibbonTrail();
  }
  
  if (m_vPosition.x < -1000.0)
  {
	  removeRibbonTrail();
	  m_vPosition.x = 1000.0; 
	  AIUnitNode->setPosition( m_vPosition );
	  addRibbonTrail();
  }

  if (m_vPosition.y < -1000.0) 
  {
	  removeRibbonTrail();
	  m_vPosition.y = 1000.0;
		AIUnitNode->setPosition( m_vPosition );
		addRibbonTrail();
  }

  if (m_vPosition.y > 1000.0)
  {
	  removeRibbonTrail();
	  m_vPosition.y = -1000.0;
	  AIUnitNode->setPosition( m_vPosition );
	  addRibbonTrail();
  }

  if (m_vPosition.z < -1000.0) 
  {
	  removeRibbonTrail();
	  m_vPosition.z = 1000.0;
	  AIUnitNode->setPosition( m_vPosition );
	  addRibbonTrail();
  }

  if (m_vPosition.z > 1000.0) 
  {
	  removeRibbonTrail();
	  m_vPosition.z = -1000.0; 
	  AIUnitNode->setPosition( m_vPosition );
	  addRibbonTrail();
  }

  
  //TODO: Still not 100% happy with rotation behaviour - 
 
  //work outcurrent orientation/facing vector
  Vector3 srcVec = AIUnitNode->getOrientation( ) * Vector3::UNIT_Z;
  srcVec.normalise();
  
  //update the heading if the vehicle has a non zero velocity
  //beware accuracy issues with doubles/floats
  if ((double)m_vVelocity.squaredLength() > 0.0000001 )
  {    
	m_vHeading = m_vVelocity.normalisedCopy();
    m_vSide = m_vHeading.perpendicular(); 
    
	Quaternion quat = Quaternion::ZERO ;
	quat= srcVec.getRotationTo( m_vHeading, Vector3::UNIT_Y );
	// i think this is needed
	quat.normalise();
	 
	//TODO: need to leave this in for now -remove asap though
	//slerp from srcvec to heading over n frames...
	//Ogre::Quaternion mOrientSrc = Quaternion::ZERO;               // Initial orientation
	//Ogre::Quaternion mOrientDest = Quaternion::ZERO;              // Destination orientation
    //Ogre::Real mRotProgress;                   // How far we've interpolated
    //Ogre::Real mRotFactor;                     // Interpolation step
	//bool mRotating;

	//mRotating = true;
	//mRotFactor = 1.0f / 1; // replace 60 with desired number of frames
	//mOrientSrc = AIUnitNode->getOrientation();
	//mOrientDest = quat * mOrientSrc;           // We want dest orientation, not a relative rotation (quat)
	//mRotProgress = 0;

	//if(mRotating)                                // Process timed rotation
//	{
//		mRotProgress += mRotFactor;
//		if(mRotProgress>1)
//		{
//		mRotating = false;
//		}
//		else
//		{
			Radian angle = Math::ACos( (double)srcVec.dotProduct(m_vHeading) );

			//if you try to rotate 180 degrees in a quaternion it bails out
			//and the mesh dissapears for some reason
			//quaternion bug/quirk workaround
			if((Degree(angle) > Degree(179)) )
			{
			AIUnitNode->yaw(Degree(180));
			//DebugText = "using Yaw to avoid mesh vanishing error";
		
			}
			else
			{
			//this	  
			// Quaternion delta = Quaternion::Slerp(mRotProgress, mOrientSrc, mOrientDest, true);
			// AIUnitNode->setOrientation(delta);

			//or this
			AIUnitNode->rotate(quat, Node::TS_WORLD);
			//AIUnitNode->setPosition( m_vPosition );
			}
//		}
//	}  // if mRotating
		
  }

 //at present always update node position
 //AIUnitNode->setPosition( m_vPosition );
}

//----------------------------- InitializeMesh --------------------------------
// loads in the mesh to be used for the unit
// needs changed to load by name perhaps(later) 
// maybe bulk out into a class?
//-----------------------------------------------------------------------------
void AIVehicle::addMeshEntity(Vector3 position, double scale) // take in inital scale,position,heading etc here?
{
	//razor mesh loads facing along negative Z axis

	Vector3 vScale(scale,scale,scale);  // mesh scale
	
	//select which mesh to use and what its name is (from individual id)
	AIEntity = AISceneMgr->createEntity("ent" + StringConverter::toString(ID()),"RZR-002.mesh" );

	//attatch node as a subnode of root
	AIUnitNode = AISceneMgr->getRootSceneNode()->createChildSceneNode();
	
	//attatch the mesh to unitnode
	AIUnitNode->attachObject( AIEntity );
	
	//set node (and hence mesh)scale
	AIUnitNode->setScale(vScale);

	// so lighting works
	AIEntity->setNormaliseNormals(true);

	AIUnitNode->setVisible(true);
	//AIUnitNode->setInheritOrientation(false);

	//set position in world
	AIUnitNode->setPosition( m_vPosition );
	//AIUnitNode->setFixedYawAxis(true, Vector3::UNIT_Y);

	//add the ribbon trail effect
	addRibbonTrail();

	//get mesh radius - use to create a bounding sphere
	SetRadius(AIEntity->getBoundingRadius());

	//set random inital facing dir
	//randomly pick a value from +/- 0-360 then convert it to radians, use it to generate a quaternion
	//then rotate by that amount around Y axis
	//also set the heading to match

	int randomAngle = (int)Math::RangeRandom(0,360);
	double angleValue = RandomClamped() * randomAngle;
	Quaternion initRot = Quaternion::ZERO;
	initRot.FromAngleAxis( Degree(angleValue) , Vector3::UNIT_Y);
	AIUnitNode->rotate(initRot, Node::TS_WORLD);
	
	//which one?!
	//Vector3 srcVec = AIUnitNode->getOrientation() * Vector3::NEGATIVE_UNIT_Z;
	Vector3 srcVec = AIUnitNode->getOrientation() * Vector3::UNIT_Z;
	SetHeading(srcVec);
	////////////////////////////////////////////////////////


	///////////////////////////

	//AIUnitNode->showBoundingBox(true);

	//use somehow?
	//AIEntity->mWorldBoundingSphere

	//create a sphere, now add more to it...
	//CreateSphere("Sphere" ,16); //atm created once per vehicle, only need 1 in total 
	//using a custom sphere class will sort that
	//Entity* sphereEntity = AISceneMgr->createEntity("SphereEntity"+ StringConverter::toString(ID()) , "Sphere");
	//SceneNode* sphereNode = AIUnitNode->createChildSceneNode();
	//sphereEntity->setMaterialName("Simple/Translucent"); //a problem with this? seems graded/random shades
	//sphereNode->attachObject(sphereEntity);
	//sphereEntity->setNormaliseNormals(true);
	//sphereNode->setVisible(false);

	//sets model facing back along -Z : not really needed i think in this case
	AIUnitNode->setDirection(Vector3::UNIT_Z,AIUnitNode->TS_WORLD, Vector3::UNIT_Z);
}

void AIVehicle::removeMeshEntity()
{
	AIUnitNode->setVisible(false);
	AIUnitNode->detachAllObjects();

	//still need to get rid of the trail
	removeRibbonTrail();

	Ogre::String entName = AIEntity->getName();
	AISceneMgr->destroyEntity(entName);
	
	Ogre::String nodeName = AIUnitNode->getName();
	AISceneMgr->destroySceneNode(nodeName);
}

void AIVehicle::addRibbonTrail()
{
	//add a nice ribbontrail effect
	//NameValuePairList pairList;
	pairList["numberOfChains"] = "1";
	pairList["maxElements"] = "20";
		
	//trail = static_cast<RibbonTrail*>(AISceneMgr->createMovableObject("rt" + StringConverter::toString(ID()), "RibbonTrail", &pairList));
	trail = (RibbonTrail*)AISceneMgr->createMovableObject("rt" + StringConverter::toString(ID()), "RibbonTrail", &pairList);
	
	trail->setMaterialName("Examples/LightRibbonTrail");
	trail->setTrailLength(80);
	trail->setVisible(true);

	//TODO: this adds a new node - check if old node getting deleted properly?
	trailNode = AISceneMgr->getRootSceneNode()->createChildSceneNode();
	trailNode->attachObject(trail);
	
	//set inital colour to orangish, then make it fade away
	trail->setInitialColour(0, 1.0, 0.8, 0);
	trail->setColourChange(0, 1.25, 1.25, 1.25, 1.25);
	
	//set initial trail width & make it taper off
	trail->setInitialWidth(0, 6);
	trail->setWidthChange(0, 1.0);

	//make the trail follow the vehicle
	trail->addNode(AIUnitNode);

}

void AIVehicle::removeRibbonTrail()
{
	trailNode->setVisible(false);
	trailNode->detachAllObjects();

	//clear pairList
	pairList.clear();

	//tidy up ribbon trail
	trail->setVisible(false);
	trail->removeNode(AIUnitNode);

	//delete ribbontrail by name
	Ogre::String trailName = trail->getName();
	//DebugText = trailName;
	AISceneMgr->destroyRibbonTrail(trailName);

	//destroy trailNode
	Ogre::String nodeName = trailNode->getName();
	AISceneMgr->destroySceneNode(nodeName);

}

//------------------------- SetHeading ----------------------------------------
//  first checks that the given heading is not a vector of zero length. If the
//  new heading is valid this function sets the entity's heading and side 
//  vectors accordingly
//-----------------------------------------------------------------------------
void AIVehicle::SetHeading(Vector3 new_heading)
{
  assert( ((double)new_heading.squaredLength() - 1.0) < 0.0001);
  
  m_vHeading = new_heading;

  //the side vector must always be perpendicular to the heading
  m_vSide = m_vHeading.perpendicular();
}




/*
// to create a sphere mesh object - from ogre wiki http://www.ogre3d.org/wiki/index.php/ManualSphereMeshes
// possibly a now out of date method - see Ogre::ManualObject
void AIVehicle::CreateSphere(const std::string& strName,const float r, const int nRings, const int nSegments)
{
	MeshPtr pSphere = MeshManager::getSingleton().createManual(strName, ResourceGroupManager::DEFAULT_RESOURCE_GROUP_NAME);
	SubMesh *pSphereVertex = pSphere->createSubMesh();

	pSphere->sharedVertexData = new VertexData();
	VertexData* vertexData = pSphere->sharedVertexData;

	// define the vertex format
	VertexDeclaration* vertexDecl = vertexData->vertexDeclaration;
	size_t currOffset = 0;
	// positions
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_POSITION);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// normals
	vertexDecl->addElement(0, currOffset, VET_FLOAT3, VES_NORMAL);
	currOffset += VertexElement::getTypeSize(VET_FLOAT3);
	// two dimensional texture coordinates
	vertexDecl->addElement(0, currOffset, VET_FLOAT2, VES_TEXTURE_COORDINATES, 0);
	currOffset += VertexElement::getTypeSize(VET_FLOAT2);

	// allocate the vertex buffer
	vertexData->vertexCount = (nRings + 1) * (nSegments+1);
	HardwareVertexBufferSharedPtr vBuf = HardwareBufferManager::getSingleton().createVertexBuffer(vertexDecl->getVertexSize(0), vertexData->vertexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	VertexBufferBinding* binding = vertexData->vertexBufferBinding;
	binding->setBinding(0, vBuf);
	float* pVertex = static_cast<float*>(vBuf->lock(HardwareBuffer::HBL_DISCARD));

	// allocate index buffer
	pSphereVertex->indexData->indexCount = 6 * nRings * (nSegments + 1);
	pSphereVertex->indexData->indexBuffer = HardwareBufferManager::getSingleton().createIndexBuffer(HardwareIndexBuffer::IT_16BIT, pSphereVertex->indexData->indexCount, HardwareBuffer::HBU_STATIC_WRITE_ONLY, false);
	HardwareIndexBufferSharedPtr iBuf = pSphereVertex->indexData->indexBuffer;
	unsigned short* pIndices = static_cast<unsigned short*>(iBuf->lock(HardwareBuffer::HBL_DISCARD));

	float fDeltaRingAngle = (Math::PI / nRings);
	float fDeltaSegAngle = (2 * Math::PI / nSegments);
	unsigned short wVerticeIndex = 0 ;

	// Generate the group of rings for the sphere
	for( int ring = 0; ring <= nRings; ring++ ) {
		float r0 = r * sinf (ring * fDeltaRingAngle);
		float y0 = r * cosf (ring * fDeltaRingAngle);

		// Generate the group of segments for the current ring
		for(int seg = 0; seg <= nSegments; seg++) {
			float x0 = r0 * sinf(seg * fDeltaSegAngle);
			float z0 = r0 * cosf(seg * fDeltaSegAngle);

			// Add one vertex to the strip which makes up the sphere
			*pVertex++ = x0;
			*pVertex++ = y0;
			*pVertex++ = z0;

			Vector3 vNormal = Vector3(x0, y0, z0).normalisedCopy();
			*pVertex++ = vNormal.x;
			*pVertex++ = vNormal.y;
			*pVertex++ = vNormal.z;

			*pVertex++ = (float) seg / (float) nSegments;
			*pVertex++ = (float) ring / (float) nRings;

			if (ring != nRings) {
                               // each vertex (except the last) has six indices pointing to it
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex;               
				*pIndices++ = wVerticeIndex + nSegments;
				*pIndices++ = wVerticeIndex + nSegments + 1;
				*pIndices++ = wVerticeIndex + 1;
				*pIndices++ = wVerticeIndex;
				wVerticeIndex ++;
			}
		}; // end for seg
	} // end for ring

	// Unlock
	vBuf->unlock();
	iBuf->unlock();
	// Generate face list
	pSphereVertex->useSharedVertices = true;

	// the original code was missing this line:
	pSphere->_setBounds( AxisAlignedBox( Vector3(-r, -r, -r), Vector3(r, r, r) ), false );
	pSphere->_setBoundingSphereRadius(r);
        
	// this line makes clear the mesh is loaded (avoids memory leaks)
    pSphere->load();
 }
 */
