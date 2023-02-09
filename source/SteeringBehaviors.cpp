#include "SteeringBehaviors.h"
#include "Vehicle.h"
#include "AIWorld.h"
#include "misc/utils.h"
#include <cassert> // probs dont need for this app

using std::string;
using std::vector;


//------------------------- ctor -----------------------------------------
//
//------------------------------------------------------------------------
SteeringBehavior::SteeringBehavior(AIVehicle* agent):
                                  
        	 m_pVehicle(agent),
             m_iFlags(0),
             m_dDBoxLength(Prm.MinDetectionBoxLength),
             m_dWeightCohesion(Prm.CohesionWeight),
             m_dWeightAlignment(Prm.AlignmentWeight),
             m_dWeightSeparation(Prm.SeparationWeight),
             m_dWeightObstacleAvoidance(Prm.ObstacleAvoidanceWeight),
             m_dWeightWander(Prm.WanderWeight),
             m_dWeightWallAvoidance(Prm.WallAvoidanceWeight),
             m_dViewDistance(Prm.ViewDistance),
             m_dWallDetectionFeelerLength(Prm.WallDetectionFeelerLength),
             m_Feelers(3),
             m_Deceleration(normal),
             m_pTargetAgent1(NULL),
             m_pTargetAgent2(NULL),
             m_dWanderDistance(WanderDist),
             m_dWanderJitter(WanderJitterPerSec),
             m_dWanderRadius(WanderRad),
             m_dWaypointSeekDistSq(WaypointSeekDist*WaypointSeekDist),
             m_dWeightSeek(Prm.SeekWeight),
             m_dWeightFlee(Prm.FleeWeight),
             m_dWeightArrive(Prm.ArriveWeight),
             m_dWeightPursuit(Prm.PursuitWeight),
             m_dWeightOffsetPursuit(Prm.OffsetPursuitWeight),
             m_dWeightInterpose(Prm.InterposeWeight),
             m_dWeightHide(Prm.HideWeight),
             m_dWeightEvade(Prm.EvadeWeight),
             m_dWeightFollowPath(Prm.FollowPathWeight),
             m_SummingMethod(prioritized)

{

	//stuff for the wander behavior
	double theta = RandFloat() * TwoPi;

	//create a vector to a target position on the wander circle (2d atm)
	//m_vWanderTarget = -1* (Vector3(m_dWanderRadius * cos(theta), 0 , m_dWanderRadius * sin(theta)) ) ;

	//define a circle - need version for sphere!
	//m_vWanderTarget = Vector3(cos(theta), sin(theta) ,sin(theta));
	
	//define a circle
	m_vWanderTarget = Vector3(cos(theta), 0 ,sin(theta));
	
	//make unit circle
	m_vWanderTarget.normalise();
	//scale up to desired size
	m_vWanderTarget *= m_dWanderRadius;

	///////////////////////////////////////////////////////////////

	//create a simple Path
    m_pPath = new Path(4,-200,-200,200,200,true);
	m_pPath->LoopOn();

	/////////////////////////////////////////////////////////////////////////////////
	//TODO: move to Wander method??
	//or make a TurnOffRenderAids()
	//create sphere -> should really create it once then "clone" it!
	 tempent = m_pVehicle->getSceneMgr()->createEntity("wp" + StringConverter::toString(m_pVehicle->ID()),"geosphere4500.mesh" );
	mNode = m_pVehicle->getNode()->createChildSceneNode();

	//attatch the mesh to unitnode
	mNode->attachObject( tempent );

	tempent->setNormaliseNormals(true);
	mNode->setScale(0.015,0.015,0.015);
	mNode->setVisible(false);

	Vector3 TempVec = Vector3(0.0,0.0,(m_dWanderDistance + m_pVehicle->GetRadius() + m_dWanderRadius) );
	mNode->setPosition(TempVec);

	createCircle(m_dWanderRadius);
	///////////////////////////////////////////////////////////////
	//redo this better!!
	OffsetEnt = m_pVehicle->getSceneMgr()->createEntity("offset" + StringConverter::toString(m_pVehicle->ID()),"geosphere4500.mesh" );
	OffsetEnt->setNormaliseNormals(true);

	FFSent = m_pVehicle->getSceneMgr()->createEntity("ffs" + StringConverter::toString(m_pVehicle->ID()),"geosphere4500.mesh" );
	mOffsetPos = m_pVehicle->getSceneMgr()->getRootSceneNode()->createChildSceneNode();
	mOffsetPos->attachObject( FFSent );
	FFSent->setNormaliseNormals(true);
	mOffsetPos->setScale(0.01,0.01,0.01);
	mOffsetPos->setVisible(false);

	m_bCircleDefined = false;

}

//---------------------------------dtor ----------------------------------
SteeringBehavior::~SteeringBehavior()
{
	//tidy up
	mNode->setVisible(false);
	mCircleNode->setVisible(false);

	mNode->detachAllObjects();
	mCircleNode->detachAllObjects();

	//delete all nodes and entities - do a check to make sure not null
	m_pVehicle->getSceneMgr()->destroyEntity(OffsetEnt->getName());
	m_pVehicle->getSceneMgr()->destroyEntity(tempent->getName());
	m_pVehicle->getSceneMgr()->destroyEntity(FFSent->getName());
	
	m_pVehicle->getSceneMgr()->destroyManualObject( circle->getName() ); 
	m_pVehicle->getSceneMgr()->destroySceneNode( mNode->getName() );
	m_pVehicle->getSceneMgr()->destroySceneNode( mCircleNode->getName() ); 

}

/////////////////////////////////////////////////////////////////////////////// CALCULATE METHODS 


//----------------------- Calculate --------------------------------------
//
//  calculates the accumulated steering force according to the method set
//  in m_SummingMethod
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Calculate()
{ 
  //reset the steering force
	m_vSteeringForce = Vector3(0.0,0.0,0.0); 

  //tag neighbors if any of the following 3 group behaviors are switched on
  if (On(separation) || On(allignment) || On(cohesion))
   {
      m_pVehicle->World()->TagVehiclesWithinViewRange(m_pVehicle, m_dViewDistance);
   }
  
  switch (m_SummingMethod)
  {
  case weighted_average:
    
    m_vSteeringForce = CalculateWeightedSum(); break;

  case prioritized:

    m_vSteeringForce = CalculatePrioritized(); break;

  default:m_vSteeringForce = Vector3(0.0,0.0,0.0); 

  }//end switch

  return m_vSteeringForce;
}

//------------------------- ForwardComponent -----------------------------
//
//  returns the forward component of the steering force
//------------------------------------------------------------------------
double SteeringBehavior::ForwardComponent()
{
  return (double)m_pVehicle->GetHeading().dotProduct(m_vSteeringForce); //TODO change this
}

//--------------------------- SideComponent ------------------------------
//  returns the side component of the steering force
//------------------------------------------------------------------------
double SteeringBehavior::SideComponent()
{
  return (double)m_pVehicle->GetSide().dotProduct(m_vSteeringForce); // TODO change this
}

//--------------------- AccumulateForce ----------------------------------
//
//  This function calculates how much of its max steering force the 
//  vehicle has left to apply and then applies that amount of the
//  force to add.
//------------------------------------------------------------------------
bool SteeringBehavior::AccumulateForce(Vector3 &RunningTot,
                                       Vector3 ForceToAdd)
{
  
  //calculate how much steering force the vehicle has used so far
  double MagnitudeSoFar = (double)RunningTot.length();

  //calculate how much steering force remains to be used by this vehicle
  double MagnitudeRemaining = m_pVehicle->GetMaxForce() - MagnitudeSoFar;

  //return false if there is no more force left to use
  if (MagnitudeRemaining <= 0.0) return false;

  //calculate the magnitude of the force we want to add
  double MagnitudeToAdd = (double)ForceToAdd.length();
  
  //if the magnitude of the sum of ForceToAdd and the running total
  //does not exceed the maximum force available to this vehicle, just
  //add together. Otherwise add as much of the ForceToAdd vector is
  //possible without going over the max.
  if (MagnitudeToAdd < MagnitudeRemaining)
  {
    RunningTot += ForceToAdd;
  }
  else
  {
    //add it to the steering force
    //perhaps should be using normalisedcopy instead?
	 // RunningTot += ( (ForceToAdd.normalisedCopy()) * MagnitudeRemaining);
	
	  //try this for now
	  ForceToAdd.normalise();
	  RunningTot += (ForceToAdd * MagnitudeRemaining);
  }

  return true;
}

//---------------------- CalculatePrioritized ----------------------------
//
//  this method calls each active steering behavior in order of priority
//  and acumulates their forces until the max steering force magnitude
//  is reached, at which time the function returns the steering force 
//  accumulated to that  point
//------------------------------------------------------------------------
Vector3 SteeringBehavior::CalculatePrioritized()
{       
	//might not bee inited to zero
	Vector3 force = Vector3::ZERO ;

  // need to sort this lot out
  // will take a while
/*
  if (On(wall_avoidance))
  {
    force = WallAvoidance(m_pVehicle->World()->Walls()) *
            m_dWeightWallAvoidance;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }
   
   */

	 /*
  if (On(hide))
  {
    assert(m_pTargetAgent1 && "Hide target not assigned");

    force = Hide(m_pTargetAgent1, m_pVehicle->World()->Obstacles()) * m_dWeightHide;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }

*/
 
  if (On(obstacle_avoidance))
  {
    force = ObstacleAvoidance(m_pVehicle->World()->Obstacles()) * m_dWeightObstacleAvoidance;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }
 
  if (On(evade))
  {
    assert(m_pTargetAgent1 && "Evade target not assigned");
    
    force = Evade(m_pTargetAgent1) * m_dWeightEvade;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }
 
  if (On(flee))
  {
    force = Flee(m_pVehicle->World()->Crosshair()) * m_dWeightFlee;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }

  //group together?
    if (On(separation))
    {
      force = Separation(m_pVehicle->World()->Agents()) * m_dWeightSeparation;

      if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    if (On(allignment))
    {
      force = Alignment(m_pVehicle->World()->Agents()) * m_dWeightAlignment;

      if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }

    if (On(cohesion))
    {
      force = Cohesion(m_pVehicle->World()->Agents()) * m_dWeightCohesion;

      if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
    }
  
  if (On(seek))
  {
    force = Seek(m_pVehicle->World()->Crosshair()) * m_dWeightSeek;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }


  if (On(arrive))
  {
    force = Arrive(m_pVehicle->World()->Crosshair(), m_Deceleration) * m_dWeightArrive;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }

  if (On(wander))
  {
    force = Wander() * m_dWeightWander;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }
 
  if (On(pursuit))
  {
    assert(m_pTargetAgent1 && "pursuit target not assigned");

    force = Pursuit(m_pTargetAgent1) * m_dWeightPursuit;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }

  if (On(offset_pursuit))
  {
    assert (m_pTargetAgent1 && "pursuit target not assigned");
	assert (!m_vOffset.isZeroLength() && "No offset assigned");

    force = OffsetPursuit(m_pTargetAgent1, m_vOffset);

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }

  if (On(interpose))
  {
    assert (m_pTargetAgent1 && m_pTargetAgent2 && "Interpose agents not assigned");

    force = Interpose(m_pTargetAgent1, m_pTargetAgent2) * m_dWeightInterpose;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }

  if (On(follow_path))
  {
    force = FollowPath() * m_dWeightFollowPath;

    if (!AccumulateForce(m_vSteeringForce, force)) return m_vSteeringForce;
  }
 
  return m_vSteeringForce;
}

//---------------------- CalculateWeightedSum ----------------------------
//
//  this simply sums up all the active behaviors X their weights and 
//  truncates the result to the max available steering force before 
//  returning
//------------------------------------------------------------------------
Vector3 SteeringBehavior::CalculateWeightedSum()
{  
	//same as above
	//sort through this
	
/*
  if (On(wall_avoidance))
  {
    m_vSteeringForce += WallAvoidance(m_pVehicle->World()->Walls()) *
                         m_dWeightWallAvoidance;
  }
   
   */

	 /*
  if (On(hide))
  {
    assert(m_pTargetAgent1 && "Hide target not assigned");

    m_vSteeringForce += Hide(m_pTargetAgent1, m_pVehicle->World()->Obstacles()) * m_dWeightHide;
  }

  */

  if (On(obstacle_avoidance))
  {
    m_vSteeringForce += ObstacleAvoidance(m_pVehicle->World()->Obstacles()) * 
            m_dWeightObstacleAvoidance;
  }
 
  if (On(evade))
  {
    assert(m_pTargetAgent1 && "Evade target not assigned");
    
    m_vSteeringForce += Evade(m_pTargetAgent1) * m_dWeightEvade;
  }

  //these next three can be combined for flocking behavior (wander is
  //also a good behavior to add into this mix)
    if (On(separation))
    {
      m_vSteeringForce += Separation(m_pVehicle->World()->Agents()) * m_dWeightSeparation;
    }

    if (On(allignment))
    {
      m_vSteeringForce += Alignment(m_pVehicle->World()->Agents()) * m_dWeightAlignment;
    }

    if (On(cohesion))
    {
      m_vSteeringForce += Cohesion(m_pVehicle->World()->Agents()) * m_dWeightCohesion;
    }

  if (On(wander))
  {
    m_vSteeringForce += Wander() * m_dWeightWander;
  }

  if (On(seek))
  {
    m_vSteeringForce += Seek(m_pVehicle->World()->Crosshair()) * m_dWeightSeek;
  }

  if (On(flee))
  {
    m_vSteeringForce += Flee(m_pVehicle->World()->Crosshair()) * m_dWeightFlee; //hmmm
  }

  if (On(arrive))
  {
    m_vSteeringForce += Arrive(m_pVehicle->World()->Crosshair(), m_Deceleration) * m_dWeightArrive;
  }

  if (On(pursuit))
  {
    assert(m_pTargetAgent1 && "pursuit target not assigned");

    m_vSteeringForce += Pursuit(m_pTargetAgent1) * m_dWeightPursuit;
  }

  if (On(offset_pursuit))
  {
    assert (m_pTargetAgent1 && "pursuit target not assigned");
    assert (!m_vOffset.isZeroLength() && "No offset assigned");

    m_vSteeringForce += OffsetPursuit(m_pTargetAgent1, m_vOffset) * m_dWeightOffsetPursuit;
  }

  if (On(interpose))
  {
    assert (m_pTargetAgent1 && m_pTargetAgent2 && "Interpose agents not assigned");

    m_vSteeringForce += Interpose(m_pTargetAgent1, m_pTargetAgent2) * m_dWeightInterpose;
  }

 
  if (On(follow_path))
  {
    m_vSteeringForce += FollowPath() * m_dWeightFollowPath;
  }

  //stop steering force from going over maximum
  if( (double)m_vSteeringForce.length() > m_pVehicle->GetMaxForce() )
  {
	  m_vSteeringForce.normalise();

	  m_vSteeringForce *= m_pVehicle->GetMaxForce();

  }

  return m_vSteeringForce;
}


/////////////////////////////////////////////////////////////////////////////// START OF BEHAVIORS

//------------------------------- Seek -----------------------------------
//
//  Given a target, this behavior returns a steering force which will
//  direct the agent towards the target
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Seek(Vector3 TargetPos)
{
  Vector3 TempVec = Vector3(0.0,0.0,0.0);

  TempVec = ( TargetPos - m_pVehicle->GetPosition() );
 
  Vector3 DesiredVelocity = TempVec.normalisedCopy() * m_pVehicle->GetMaxSpeed();

 return ( DesiredVelocity - m_pVehicle->GetVelocity() ); 
}

//----------------------------- Flee -------------------------------------
//
//  Does the opposite of Seek
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Flee(Vector3 TargetPos)
{
  //only flee if the target is within 'panic distance'. Work in distance squared space. 
  //could redo this - see evade
  double ySeperation = ( (TargetPos.y ) - (m_pVehicle->GetPosition().y) ) ;
  double xSeperation = ( (TargetPos.x ) - (m_pVehicle->GetPosition().x) ) ;
  double zSeperation = ( (TargetPos.z ) - (m_pVehicle->GetPosition().z) ) ;

  double distance = ( ySeperation*ySeperation + xSeperation*xSeperation + zSeperation*zSeperation  );

  const double PanicDistanceSq = 100.0 * 100.0;
  
  if (distance < PanicDistanceSq)
  {
	m_pVehicle->DebugText = "distance < panic dist";
	
	Vector3 TempVec = Vector3(0.0,0.0,0.0);

	TempVec = (m_pVehicle->GetPosition() - TargetPos);

	Vector3 DesiredVelocity = ( ( TempVec.normalisedCopy() ) * m_pVehicle->GetMaxSpeed() ) ;

	return ( DesiredVelocity - m_pVehicle->GetVelocity() );
	}
  else
  {
   Vector3 vec = Vector3(0.0,0.0,0.0);
   return vec;
  }

}
	
//--------------------------- Arrive -------------------------------------
//
//  This behavior is similar to seek but it attempts to arrive at the
//  target with a zero velocity
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Arrive(Vector3 TargetPos, Deceleration deceleration)
{
  Vector3 ToTarget = (TargetPos - m_pVehicle->GetPosition() );

  //calculate the distance to the target
  double dist = (double)ToTarget.length();

  if (dist > 0) //
  {
    //because Deceleration is enumerated as an int, this value is required
    //to provide fine tweaking of the deceleration..
    const double DecelerationTweaker = 0.3 ; //was 0.3

    //calculate the speed required to reach the target given the desired
    //deceleration
    double speed =  dist / ((double)deceleration * DecelerationTweaker);     

    //make sure the velocity does not exceed the max

	if (speed > m_pVehicle->GetMaxSpeed())
	{
		speed = m_pVehicle->GetMaxSpeed();
	}

    //from here proceed just like Seek except we don't need to normalize 
    //the ToTarget vector because we have already gone to the trouble
    //of calculating its length: dist. 
    Vector3 DesiredVelocity =  (ToTarget * speed / dist);

	//for visual debugging- show the steering force being applied
	mOffsetPos->setVisible(true);
	mOffsetPos->setPosition(m_pVehicle->GetPosition()+ DesiredVelocity  );

    return (DesiredVelocity - m_pVehicle->GetVelocity());
  }

  return Vector3::ZERO;
}

//------------------------------ Pursuit ---------------------------------
//
//  this behavior creates a force that steers the agent towards the 
//  evader
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Pursuit(const AIVehicle* evader)
{
	
  //if the evader is ahead and facing the agent then we can just seek
  //for the evader's current position.
  Vector3 ToEvader = evader->GetPosition() - m_pVehicle->GetPosition();

  double RelativeHeading = m_pVehicle->GetHeading().dotProduct( evader->GetHeading() );

//dot product tells us if the headings are similar or not
  if ( (ToEvader.dotProduct(m_pVehicle->GetHeading()) > 0) && (RelativeHeading < -0.95))  //acos(0.95)=18 degs
  {
    return Seek(evader->GetPosition());
  }

  //Not considered ahead so we predict where the evader will be.
 
  //the lookahead time is propotional to the distance between the evader
  //and the pursuer; and is inversely proportional to the sum of the
  //agent's velocities
  double LookAheadTime = (double)ToEvader.length() / ( m_pVehicle->GetMaxSpeed() + evader->Speed() );
  
  //now seek to the predicted future position of the evader
  return Seek(evader->GetPosition() + evader->GetVelocity() * LookAheadTime);  
}

//----------------------------- Evade ------------------------------------
//
//  similar to pursuit except the agent Flees from the estimated future
//  position of the pursuer
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Evade(const AIVehicle* pursuer)
{

  //Not necessary to include the check for facing direction this time 
  Vector3 ToPursuer = pursuer->GetPosition() - m_pVehicle->GetPosition();

  //uncomment the following two lines to have Evade only consider pursuers 
  //within a 'threat range'
  const double ThreatRange = 80.0;
  if (ToPursuer.squaredLength()> ( ThreatRange * ThreatRange) ) return Vector3(0.0,0.0,0.0);
 
  //the lookahead time is propotional to the distance between the pursuer
  //and the pursuer; and is inversely proportional to the sum of the
  //agents' velocities
  double LookAheadTime = (double)ToPursuer.length() / (m_pVehicle->GetMaxSpeed() + pursuer->Speed());
  
  //now flee away from predicted future position of the pursuer
  return Flee(pursuer->GetPosition() + pursuer->GetVelocity() * LookAheadTime);
  
}

//--------------------------- Wander -------------------------------------
//
//  This behavior makes the agent wander about randomly
//  TODO: works but make it better
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Wander()
{
  //this behavior is dependent on the update rate, so this line must
  //be included when using time independent framerate.
  //double JitterThisTimeSlice = m_dWanderJitter * m_pVehicle->TimeElapsed();

//however this isnt time independent i think! and each time elasped is about 0.00166667 seconds.... 
   double JitterThisTimeSlice = m_dWanderJitter;

  //first, add a small random vector to the target's position
  m_vWanderTarget += Vector3(RandomClamped() * JitterThisTimeSlice,
							 0, //RandomClamped() * JitterThisTimeSlice, //for now
                             RandomClamped() * JitterThisTimeSlice);
			 
  //reproject this new vector back on to a unit circle
  m_vWanderTarget.normalise();

  //increase the length of the vector to the same as the radius
  //of the wander circle
  m_vWanderTarget *= m_dWanderRadius;

  //move the target into a position WanderDist in front of the agent (taking both radii into account)
  Vector3 target = m_vWanderTarget + Vector3(0.0,0.0, m_dWanderDistance+(m_pVehicle->GetRadius() + m_dWanderRadius));

  mNode->setVisible(true);
  mCircleNode->setVisible(true);
  mNode->setPosition(target); //sets target postion relative to mNodes'parent, making this in object space

  //TODO: messy this tidy it up
  Vector3 Temp4;

  Temp4 = mNode->getWorldPosition();  //equivalent to performing a worldtransform on target!
   
  Vector3 resultVec2 = (Temp4 - m_pVehicle->GetPosition());

   return resultVec2;

}


//---------------------------- Separation --------------------------------
//
// this calculates a force repelling from the other neighbors
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Separation(const vector<AIVehicle*> &neighbors)
{  
  Vector3 SteeringForce = Vector3(0.0,0.0,0.0);

  for (unsigned int a=0; a<neighbors.size(); ++a)
  {
    //make sure this agent isn't included in the calculations and that
    //the agent being examined is close enough. ***also make sure it doesn't
    //include the evade target ***
    if((neighbors[a] != m_pVehicle) && neighbors[a]->IsTagged() &&
      (neighbors[a] != m_pTargetAgent1))
    {
      Vector3 ToAgent = m_pVehicle->GetPosition() - neighbors[a]->GetPosition();

	  //this seems to work well
	  SteeringForce += ToAgent.normalisedCopy() / (double)ToAgent.length();
    }
  }

  return SteeringForce;
}

//---------------------------- Alignment ---------------------------------
//
//  returns a force that attempts to align this agents heading with that
//  of its neighbors
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Alignment(const vector<AIVehicle*>& neighbors)
{
	
  //used to record the average heading of the neighbors
  Vector3 AverageHeading = Vector3(0.0,0.0,0.0);

  //used to count the number of vehicles in the neighborhood
  int    NeighborCount = 0;

  //iterate through all the tagged vehicles and sum their heading vectors  
  for (unsigned int a=0; a<neighbors.size(); ++a)
  {
    //make sure *this* agent isn't included in the calculations and that
    //the agent being examined  is close enough ***also make sure it doesn't
    //include any evade target *** - screws up pursuit i think
    if((neighbors[a] != m_pVehicle) && neighbors[a]->IsTagged() &&
      (neighbors[a] != m_pTargetAgent1))
    {
      AverageHeading += neighbors[a]->GetHeading();

      ++NeighborCount;
    }
  }

  //if the neighborhood contained one or more vehicles, average their
  //heading vectors.
  if (NeighborCount > 0)
  {
	  //is this causing problems? - dont see why need to remove heading when it was never added?
	  AverageHeading -= m_pVehicle->GetHeading();

	  AverageHeading /= NeighborCount;	  
  }
  
  return AverageHeading;
}

//-------------------------------- Cohesion ------------------------------
//
//  returns a steering force that attempts to move the agent towards the
//  center of mass of the agents in its immediate area
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Cohesion(const vector<AIVehicle*> &neighbors)
{
	
  //first find the center of mass of all the agents

  Vector3 CenterOfMass, SteeringForce ;

  CenterOfMass = Vector3(0.0,0.0,0.0);
  SteeringForce = Vector3(0.0,0.0,0.0);

  int NeighborCount = 0;

  //iterate through the neighbors and sum up all the position vectors
  for (unsigned int a=0; a<neighbors.size(); ++a)
  {
    //make sure *this* agent isn't included in the calculations and that
    //the agent being examined is close enough ***also make sure it doesn't
    //include the evade target ***
    if((neighbors[a] != m_pVehicle) && neighbors[a]->IsTagged() &&
      (neighbors[a] != m_pTargetAgent1))
    {
      CenterOfMass += neighbors[a]->GetPosition();

      ++NeighborCount;
    }
  }

  if (NeighborCount > 0)
  {
    //the center of mass is the average of the sum of positions
    CenterOfMass /= NeighborCount;

    //now seek towards that position
    SteeringForce = Seek(CenterOfMass);
  }

  //the magnitude of cohesion is usually much larger than separation or
  //allignment so it usually helps to normalize it.

  return SteeringForce.normalisedCopy();

}


//--------------------------- Interpose ----------------------------------
//
//  Given two agents, this method returns a force that attempts to 
//  position the vehicle between them
//------------------------------------------------------------------------
Vector3 SteeringBehavior::Interpose(const AIVehicle* AgentA,
                                     const AIVehicle* AgentB)
{
	
  //first we need to figure out where the two agents are going to be at 
  //time T in the future. This is approximated by determining the time
  //taken to reach the mid way point at the current time at at max speed.
  Vector3 MidPoint = (AgentA->GetPosition() + AgentB->GetPosition()) / 2.0;

  double ySeperation = ( (MidPoint.y ) - (m_pVehicle->GetPosition().y) ) ;
  double xSeperation = ( (MidPoint.x ) - (m_pVehicle->GetPosition().x) ) ;
  double zSeperation = ( (MidPoint.z ) - (m_pVehicle->GetPosition().z) ) ;

  double distance = sqrt(ySeperation*ySeperation + xSeperation*xSeperation + zSeperation*zSeperation  );

  double TimeToReachMidPoint = distance / m_pVehicle->GetMaxSpeed();

  //now we have T, we assume that agent A and agent B will continue on a
  //straight trajectory and extrapolate to get their future positions
  Vector3 APos = AgentA->GetPosition() + AgentA->GetVelocity() * TimeToReachMidPoint;
  Vector3 BPos = AgentB->GetPosition() + AgentB->GetVelocity() * TimeToReachMidPoint;

  //calculate the mid point of these predicted positions
  MidPoint = (APos + BPos) / 2.0;

  //then steer to Arrive at it
  return Arrive(MidPoint, fast);
  
}

//------------------------------- FollowPath -----------------------------
//
//  Given a series of Vector2Ds, this method produces a force that will
//  move the agent along the waypoints in order. The agent uses the
// 'Seek' behavior to move to the next waypoint - unless it is the last
//  waypoint, in which case it 'Arrives'
//------------------------------------------------------------------------
Vector3 SteeringBehavior::FollowPath()
{ 
  //fix this
  //move to next target if close enough to current target (working in
  //distance squared space)
  double ySeperation = ( (m_pVehicle->GetPosition().y) - (m_pPath->CurrentWaypoint().y) ) ;
  double xSeperation = ( (m_pVehicle->GetPosition().x) - (m_pPath->CurrentWaypoint().x) ) ;
  double zSeperation = ( (m_pVehicle->GetPosition().z) - (m_pPath->CurrentWaypoint().z) ) ;
  
  double distance = sqrt(ySeperation*ySeperation + xSeperation*xSeperation + zSeperation*zSeperation  );

  if( distance <  m_dWaypointSeekDistSq)
  {
    m_pPath->SetNextWaypoint();
  }

  if (!m_pPath->Finished())
  {
    return Seek(m_pPath->CurrentWaypoint());
  }

  else
  {
    return Arrive(m_pPath->CurrentWaypoint(), normal);
  }
  
}


//------------------------- Offset Pursuit -------------------------------
//
//  Produces a steering force that keeps a vehicle at a specified offset
//  from a leader vehicle
//------------------------------------------------------------------------
Vector3 SteeringBehavior::OffsetPursuit(const AIVehicle* leader, const Vector3 offset)
{

Vector3 TargetPos = mOffsetNode->getWorldPosition();

 //Vector3 ToTarget = mOffsetNode->getWorldPosition() - m_pVehicle->GetPosition() ;
 //Vector3 ToTarget = mOffsetNode->_getDerivedPosition() - m_pVehicle->GetPosition() ;

	Vector3 ToTarget = TargetPos - m_pVehicle->GetPosition();

  double dist = (double)ToTarget.length(); 
  
  if(dist > 0)
  {
	const double DecelerationTweaker = 1.5 ; //was 0.3

    //calculate the speed required to reach the target given the desired
    //deceleration
    double speed =  dist / ((double)normal * DecelerationTweaker); 
	
	if (speed > m_pVehicle->GetMaxSpeed())
	{
		speed = m_pVehicle->GetMaxSpeed();
	}
	 //speed = min(speed, m_pVehicle->GetMaxSpeed());

	Vector3 DesiredVelocity =  ToTarget * (speed  / dist) ;

	mOffsetPos->setVisible(true);
	mOffsetPos->setPosition( m_pVehicle->GetPosition()+ DesiredVelocity );

	Vector3 TempVec = DesiredVelocity  - m_pVehicle->GetVelocity();
	return (TempVec);
  }

  return Vector3(0.0,0.0,0.0);
}

/*
//------------------------- Offset Pursuit -------------------------------
//
//  Produces a steering force that keeps a vehicle at a specified offset
//  from a leader vehicle
//------------------------------------------------------------------------
Vector3 SteeringBehavior::OffsetPursuit(const AIVehicle* leader, const Vector3 offset)
{
  Vector3 OffsetPos = mOffsetNode->getWorldPosition();
  
  Vector3 ToOffset = OffsetPos - m_pVehicle->GetPosition() ;
  
  //im sure this is wrong
  double LookAheadTime = (double)ToOffset.length() / ( m_pVehicle->GetMaxSpeed() + leader->Speed() );
  
  //predict future position based on look ahead time
   //return Arrive(OffsetPos + leader->GetVelocity() * LookAheadTime, slow);
  //ArriveOn(); 
  //return Arrive( m_pVehicle->World()->Crosshair() , normal);
 }
*/

void SteeringBehavior::OffsetEnterHelper(AIVehicle* leader,Vector3 offset)
{
	mOffsetNode = leader->getNode()->createChildSceneNode( "offsetnode" + StringConverter::toString(m_pVehicle->ID()) );
	mOffsetNode->attachObject( OffsetEnt );
	mOffsetNode->setScale(0.015,0.015,0.015);
	mOffsetNode->setVisible(true);
	mOffsetNode->setPosition(offset);
}

void SteeringBehavior::OffsetExitHelper()
{
	mOffsetNode->setVisible(false);
	mOffsetNode->detachAllObjects();
	m_pVehicle->getSceneMgr()->destroySceneNode( mOffsetNode->getName() );

}


//---------------------- ObstacleAvoidance -------------------------------
//
//  Given a vector of CObstacles, this method returns a steering force
//  that will prevent the agent colliding with the closest obstacle
//------------------------------------------------------------------------
Vector3 SteeringBehavior::ObstacleAvoidance(const std::vector<BaseGameEntity*>& obstacles)
{
 // mExtendedRadiusNode->setVisible(true);
  mOffsetPos->setVisible(true);

  //the detection box length is proportional to the agent's velocity
  m_dDBoxLength = Prm.MinDetectionBoxLength + ( m_pVehicle->Speed()/m_pVehicle->GetMaxSpeed() ) * Prm.MinDetectionBoxLength;

  //set the visual aid for detectionbox
  ObstacleExecuteHelper(m_dDBoxLength);
 
  //tag all obstacles within range of the box for processing
  m_pVehicle->World()->TagObstaclesWithinViewRange(m_pVehicle, m_dDBoxLength);

  //this will keep track of the closest intersecting obstacle (CIB)
  BaseGameEntity* ClosestIntersectingObstacle = NULL;
 
  //this will be used to track the distance to the CIB
  double DistToClosestIP = MaxDouble;

  //this will record the transformed local coordinates of the CIB
  Vector3 LocalPosOfClosestObstacle;

  std::vector<BaseGameEntity*>::const_iterator curOb = obstacles.begin();

  //////////////////////////////////////////////////////////////////////////
  while(curOb != obstacles.end())
  {
    //if the obstacle has been tagged within range proceed
    if ( (*curOb)->IsTagged() )
    {

    //world space positions for Unit & obstacle
	Vector3 UnitPos = m_pVehicle->getNode()->getWorldPosition();
	Vector3 Objectpos = (*curOb)->mNode->getWorldPosition(); 

	//*should* be this im sure (object world to vehicle local)
	Vector3 LocalPos = ( m_pVehicle->getNode()->_getFullTransform().inverse() * (*curOb)->mNode->getWorldPosition() ) ;

	LocalPos *= 0.5; //needed as vals seemed to be double whats expected

	mOffsetPos->setPosition(Objectpos);

	  //if the local position has a negative z value then it must lay
      //behind the agent. (in which case it can be ignored)

     //TODO: double check "handedness" of original code
	
	  if (LocalPos.z >= 0)
      {
        //if the distance from the z axis to the object's position is less
        //than its radius + half the width of the detection box then there
        //is a potential intersection.

        double ExpandedRadius = (*curOb)->GetRadius() + m_pVehicle->GetRadius();
		
	//Visual Debugging- draw a circle showing the expanded radius - to be removed
	//////////////////////////////////////////////////////////////////
	if(m_bCircleDefined == false)
	{

		obscircle = m_pVehicle->getSceneMgr()->createManualObject("expandedcircle" + StringConverter::toString( m_pVehicle->ID()));
	
    float const radius = ExpandedRadius,
                thickness = 1.0, // Of course this must be less than the radius value.
                accuracy = 5;

    obscircle->begin("BaseWhiteNoLighting", RenderOperation::OT_TRIANGLE_LIST);

    unsigned point_index = 0;
    for(float theta = 0; theta <= 2 * Math::PI; theta += Math::PI / (radius * accuracy)) {
        obscircle->position(radius * cos(theta),
                         0,
                         radius * sin(theta));
        obscircle->position(radius * cos(theta - Math::PI / (radius * accuracy)),
                         0,
                         radius * sin(theta - Math::PI / (radius * accuracy)));
        obscircle->position((radius - thickness) * cos(theta - Math::PI / (radius * accuracy)),
                         0,
                         (radius - thickness) * sin(theta - Math::PI / (radius * accuracy)));
        obscircle->position((radius - thickness) * cos(theta),
                         0,
                         (radius - thickness) * sin(theta));
        // Join the 4 vertices created above to form a quad.
        obscircle->quad(point_index, point_index + 1, point_index + 2, point_index + 3);
        point_index += 4;
    }

    obscircle->end();
	
	mExtendedRadiusNode = m_pVehicle->getSceneMgr()->getRootSceneNode()->createChildSceneNode();

	mExtendedRadiusNode->detachAllObjects();
	mExtendedRadiusNode->attachObject(obscircle);
	m_bCircleDefined = true;
	}

    /////////////////////////////////////////////////////////////////////////
	 
	if ( LocalPos.x < ExpandedRadius)
        {
		  //replace with a better line/circle test! - using ogre or ogrenewt!
		  //better yet line/sphere or box/sphere

          //now to do a line/circle intersection test. The center of the 
          //circle is represented by (cX, cY). The intersection points are 
          //given by the formula x = cX +/-sqrt(r^2-cY^2) for y=0. 
          //We only need to look at the smallest positive value of x because
          //that will be the closest point of intersection.
          
		  double cX = LocalPos.x;
          double cZ = LocalPos.z;
		 
          //we only need to calculate the sqrt part of the above equation once
		  double SqrtPart = sqrt(ExpandedRadius*ExpandedRadius - cX*cX);

		  double ip = cZ - SqrtPart;

		  if (ip <= 0.0)
          {
            ip = cZ + SqrtPart;
          }

          //test to see if this is the closest so far. If it is keep a
          //record of the obstacle and its local coordinates
          if ( ip < DistToClosestIP )
          {
            DistToClosestIP = ip;

            ClosestIntersectingObstacle = (*curOb) ;

            //LocalPosOfClosestObstacle = LocalPos;
			  LocalPosOfClosestObstacle = (*curOb)->GetPosition();
          }         
        }
      }
	 
    }

    ++curOb;
  }

  ///////////////////////////////////////////////////////////////

  //if we have found an intersecting obstacle, calculate a steering 
  //force away from it
  Vector3 SteeringForce = Vector3(0.0,0.0,0.0) ;
  
  if (ClosestIntersectingObstacle)
  {

	mExtendedRadiusNode->setPosition( ClosestIntersectingObstacle->GetPosition() );
	mOffsetPos->setPosition( ClosestIntersectingObstacle->GetPosition() );
	
    //the closer the agent is to an object, the stronger the steering force should be
	double multiplier = 1.0 + (m_dDBoxLength - LocalPosOfClosestObstacle.z) / m_dDBoxLength;

    //calculate the lateral force
	SteeringForce.x = (ClosestIntersectingObstacle->GetRadius() - LocalPosOfClosestObstacle.x)  * multiplier ;

    //apply a braking force proportional to the obstacles distance from the vehicle. 
	const double BrakingWeight = 0.9;

	SteeringForce.z = (ClosestIntersectingObstacle->GetRadius() - LocalPosOfClosestObstacle.z) * BrakingWeight ;
	
	//SteeringForce.y = (ClosestIntersectingObstacle->GetRadius() - LocalPosOfClosestObstacle.y)  * multiplier ;
	SteeringForce.y =  0.0;
  }

 return SteeringForce;
}

void SteeringBehavior::ObstacleEnterHelper()
{
	//node used to show detection box length
	mOffsetNode = m_pVehicle->getNode()->createChildSceneNode( "obstaclenode" + StringConverter::toString(m_pVehicle->ID()) );
	mOffsetNode->attachObject( OffsetEnt );
	mOffsetNode->setScale(0.015,0.015,0.015);
	mOffsetNode->setVisible(true);

	//node used to show position of ...
	//mOffsetPos = m_pVehicle->getNode()->createChildSceneNode( "posnode" + StringConverter::toString(m_pVehicle->ID()) );
	
	//m_pVehicle->getSceneMgr()->getRootSceneNode()->removeChild(mOffsetPos->getName());
	m_pVehicle->getSceneMgr()->destroySceneNode( mOffsetPos->getName() );
	mOffsetPos = m_pVehicle->getSceneMgr()->getRootSceneNode()->createChildSceneNode( "posnode" + StringConverter::toString(m_pVehicle->ID()) );
	mOffsetPos->attachObject( FFSent );
	mOffsetPos->setScale(0.01,0.01,0.01);
	mOffsetPos->setVisible(true);
}

void SteeringBehavior::ObstacleExecuteHelper(double offset)
{
	Vector3 BoxLength = Vector3(0.0,0.0,offset);
	mOffsetNode->setPosition(BoxLength);
}

void SteeringBehavior::ObstacleExitHelper()
{
	mExtendedRadiusNode->setVisible(false);
	mOffsetNode->setVisible(false);
	mOffsetNode->detachAllObjects();
	m_pVehicle->getSceneMgr()->destroySceneNode( mOffsetNode->getName() );

	mOffsetPos->setVisible(false);
	mOffsetPos->detachAllObjects();
	m_pVehicle->getSceneMgr()->destroySceneNode( mOffsetPos->getName() );

}




/* // WALLS = implement as TERRAIN mesh or just any mesh inc entities??
   // OR as a manual object?
//--------------------------- WallAvoidance --------------------------------
//
//  This returns a steering force that will keep the agent away from any
//  walls it may encounter
//------------------------------------------------------------------------
Vector2D SteeringBehavior::WallAvoidance(const std::vector<Wall2D>& walls)
{
	
  //the feelers are contained in a std::vector, m_Feelers
  CreateFeelers();
  
  double DistToThisIP    = 0.0;
  double DistToClosestIP = MaxDouble;

  //this will hold an index into the vector of walls
  int ClosestWall = -1;

  Vector2D SteeringForce,
            point,         //used for storing temporary info
            ClosestPoint;  //holds the closest intersection point

  //examine each feeler in turn
  for (unsigned int flr=0; flr<m_Feelers.size(); ++flr)
  {
    //run through each wall checking for any intersection points
    for (unsigned int w=0; w<walls.size(); ++w)
    {
      if (LineIntersection2D(m_pVehicle->Pos(),
                             m_Feelers[flr],
                             walls[w].From(),
                             walls[w].To(),
                             DistToThisIP,
                             point))
      {
        //is this the closest found so far? If so keep a record
        if (DistToThisIP < DistToClosestIP)
        {
          DistToClosestIP = DistToThisIP;

          ClosestWall = w;

          ClosestPoint = point;
        }
      }
    }//next wall

  
    //if an intersection point has been detected, calculate a force  
    //that will direct the agent away
    if (ClosestWall >=0)
    {
      //calculate by what distance the projected position of the agent
      //will overshoot the wall
      Vector2D OverShoot = m_Feelers[flr] - ClosestPoint;

      //create a force in the direction of the wall normal, with a 
      //magnitude of the overshoot
      SteeringForce = walls[ClosestWall].Normal() * OverShoot.Length();
    }

  }//next feeler

  
  return SteeringForce;
}
*/


/*
//------------------------------- CreateFeelers --------------------------
//
//  Creates the antenna utilized by WallAvoidance
//------------------------------------------------------------------------
void SteeringBehavior::CreateFeelers()
{
	
  //feeler pointing straight in front
  m_Feelers[0] = m_pVehicle->Pos() + m_dWallDetectionFeelerLength * m_pVehicle->Heading();

  //feeler to left
  Vector2D temp = m_pVehicle->Heading();
  Vec2DRotateAroundOrigin(temp, HalfPi * 3.5f);
  m_Feelers[1] = m_pVehicle->Pos() + m_dWallDetectionFeelerLength/2.0f * temp;

  //feeler to right
  temp = m_pVehicle->Heading();
  Vec2DRotateAroundOrigin(temp, HalfPi * 0.5f);
  m_Feelers[2] = m_pVehicle->Pos() + m_dWallDetectionFeelerLength/2.0f * temp;
  
}

*/

//--------------------------- Hide ---------------------------------------
//
//------------------------------------------------------------------------
//Vector2D SteeringBehavior::Hide(const Vehicle*           hunter,
//                                 const vector<BaseGameEntity*>& obstacles)
//{
	/*
  double    DistToClosest = MaxDouble;
  Vector2D BestHidingSpot;

  std::vector<BaseGameEntity*>::const_iterator curOb = obstacles.begin();
  std::vector<BaseGameEntity*>::const_iterator closest;

  while(curOb != obstacles.end())
  {
    //calculate the position of the hiding spot for this obstacle
    Vector2D HidingSpot = GetHidingPosition((*curOb)->Pos(),
                                             (*curOb)->BRadius(),
                                              hunter->Pos());
            
    //work in distance-squared space to find the closest hiding
    //spot to the agent
    double dist = Vec2DDistanceSq(HidingSpot, m_pVehicle->Pos());

    if (dist < DistToClosest)
    {
      DistToClosest = dist;

      BestHidingSpot = HidingSpot;

      closest = curOb;
    }  
            
    ++curOb;

  }//end while
  
  //if no suitable obstacles found then Evade the hunter
  if (DistToClosest == MaxFloat)
  {
    return Evade(hunter);
  }
      
  //else use Arrive on the hiding spot
  return Arrive(BestHidingSpot, fast);
  */
//}

/*
//------------------------- GetHidingPosition ----------------------------
//
//  Given the position of a hunter, and the position and radius of
//  an obstacle, this method calculates a position DistanceFromBoundary 
//  away from its bounding radius and directly opposite the hunter
//------------------------------------------------------------------------
Vector3 SteeringBehavior::GetHidingPosition(const Vector3& posOb,
                                              const double     radiusOb,
                                              const Vector3& posHunter)
{
	
  //calculate how far away the agent is to be from the chosen obstacle's
  //bounding radius
  const double DistanceFromBoundary = 30.0;
  double       DistAway    = radiusOb + DistanceFromBoundary;

  //calculate the heading toward the object from the hunter
  //Vector2D ToOb = Vec2DNormalize(posOb - posHunter);
  Vector3 TempVec = (posOb - posHunter);
  Vector3 ToOb = TempVec.normalisedCopy();
  
  //scale it to size and add to the obstacles position to get
  //the hiding spot.
  return (ToOb * DistAway) + posOb;
  
}
*/

//circle code taken from http://www.ogre3d.org/wiki/index.php/Circle
void SteeringBehavior::createCircle(double dRadius)
{
	//TODO: replace with something else later on ( toriod?, sphere?)
	
	// for 3d circle with thickness
	circle = m_pVehicle->getSceneMgr()->createManualObject("circle" + StringConverter::toString( m_pVehicle->ID()));

    float const radius = dRadius,
                thickness = 1.0, // Of course this must be less than the radius value.
                accuracy = 10;

    //add in test to make sure thickness is less and if not make it so....

    circle->begin("BaseWhiteNoLighting", RenderOperation::OT_TRIANGLE_LIST);

    unsigned point_index = 0;
    for(float theta = 0; theta <= 2 * Math::PI; theta += Math::PI / (radius * accuracy)) {
        circle->position(radius * cos(theta),
                         0,
                         radius * sin(theta));
        circle->position(radius * cos(theta - Math::PI / (radius * accuracy)),
                         0,
                         radius * sin(theta - Math::PI / (radius * accuracy)));
        circle->position((radius - thickness) * cos(theta - Math::PI / (radius * accuracy)),
                         0,
                         (radius - thickness) * sin(theta - Math::PI / (radius * accuracy)));
        circle->position((radius - thickness) * cos(theta),
                         0,
                         (radius - thickness) * sin(theta));
        // Join the 4 vertices created above to form a quad.
        circle->quad(point_index, point_index + 1, point_index + 2, point_index + 3);
        point_index += 4;
    }

    circle->end();


	//mCircleNode =  m_pVehicle->getNode()->createChildSceneNode(Vector3(0.0,0.0,m_dWanderRadius + m_pVehicle->GetRadius() ));
	mCircleNode =  m_pVehicle->getNode()->createChildSceneNode();
	mCircleNode->attachObject(circle);

	mCircleNode->setInheritOrientation(true);
	Vector3 TempVec = Vector3(0.0,0.0,(m_dWanderDistance + m_pVehicle->GetRadius() + m_dWanderRadius) );
	
	mCircleNode->setPosition(TempVec);
	mCircleNode->setVisible(false);

}

