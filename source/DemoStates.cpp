#include "DemoStates.h"
#include "State.h"
#include "AIWorld.h"

//////////////////////////////////////////////////

IdleDemo* IdleDemo::Instance()
{
  static IdleDemo instance;

  return &instance;
}


void IdleDemo::Enter(AIWorld* pWorld)
{
 //do nothing 
}

void IdleDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void IdleDemo::Exit(AIWorld* pWorld)
{
  //do nothing 
}

//////////////////////////////////////////////////

SeekDemo* SeekDemo::Instance()
{
  static SeekDemo instance;

  return &instance;
}


void SeekDemo::Enter(AIWorld* pWorld)
{
 //load n number of ships
 //seek towards target

	pWorld->AddVehicle();
	pWorld->Agents().at(0)->Steering()->SeekOn();

}

void SeekDemo::Execute(AIWorld* pWorld)
{  
  //continue seeking 

}


void SeekDemo::Exit(AIWorld* pWorld)
{
  //stop seeking
  pWorld->Agents().at(0)->Steering()->SeekOff();
  
  //unload ship
  //pWorld->Agents().at(0)->removeMeshEntity();

  //clear agent list
  pWorld->RemoveAllVehicles();

}

//////////////////////////////////////////////////

FleeDemo* FleeDemo::Instance()
{
  static FleeDemo instance;

  return &instance;
}


void FleeDemo::Enter(AIWorld* pWorld)
{
 	pWorld->AddVehicle();
	pWorld->Agents().at(0)->Steering()->FleeOn();
}

void FleeDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void FleeDemo::Exit(AIWorld* pWorld)
{
  //stop seeking
  pWorld->Agents().at(0)->Steering()->FleeOff();
  
  //unload ship
  //pWorld->Agents().at(0)->removeMeshEntity();

  //clear agent list
  pWorld->RemoveAllVehicles();
}

//////////////////////////////////////////////////

ArriveDemo* ArriveDemo::Instance()
{
  static ArriveDemo instance;

  return &instance;
}


void ArriveDemo::Enter(AIWorld* pWorld)
{
	//set up 1 agent
	pWorld->AddVehicle();

	//turn on arrive behaviour
	pWorld->Agents().at(0)->Steering()->ArriveOn();
	

}

void ArriveDemo::Execute(AIWorld* pWorld)
{  
  
	//run agents update methods
}


void ArriveDemo::Exit(AIWorld* pWorld)
{
 //stop arriving
  pWorld->Agents().at(0)->Steering()->ArriveOff();
  
  
  //unload ship
  //pWorld->Agents().at(0)->removeMeshEntity();

  //clear agent list
  pWorld->RemoveAllVehicles();

}

//////////////////////////////////////////////////

EvadeDemo* EvadeDemo::Instance()
{
  static EvadeDemo instance;

  return &instance;
}


void EvadeDemo::Enter(AIWorld* pWorld)
{
 //add two units
	for(int i = 0; i<2; i++)
	{
		pWorld->AddVehicle();
	};

	pWorld->Agents().at(0)->Steering()->PursuitOn(pWorld->Agents().at(1));
	pWorld->Agents().at(1)->Steering()->EvadeOn(pWorld->Agents().at(0));
}

void EvadeDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void EvadeDemo::Exit(AIWorld* pWorld)
{
   //turn off behaviours
	pWorld->Agents().at(0)->Steering()->PursuitOff();
	pWorld->Agents().at(1)->Steering()->EvadeOff();

	//destroy entities
	for(int i = 0; i<2; i++)
	{
		//pWorld->Agents().at(i)->removeMeshEntity();
	};

	 //clear agent list
  pWorld->RemoveAllVehicles();	
}

//////////////////////////////////////////////////

PursuitDemo* PursuitDemo::Instance()
{
  static PursuitDemo instance;

  return &instance;
}


void PursuitDemo::Enter(AIWorld* pWorld)
{
	//add two units
	for(int i = 0; i<2; i++)
	{
		pWorld->AddVehicle();
	};

	pWorld->Agents().at(0)->Steering()->WanderOn();
	pWorld->Agents().at(1)->Steering()->PursuitOn(pWorld->Agents().at(0));

}

void PursuitDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void PursuitDemo::Exit(AIWorld* pWorld)
{
  //turn off behaviours
	pWorld->Agents().at(0)->Steering()->WanderOff();
	pWorld->Agents().at(1)->Steering()->PursuitOff();

	//destroy entities
	for(int i = 0; i<2; i++)
	{
		//pWorld->Agents().at(i)->removeMeshEntity();
	};

	 //clear agent list
  pWorld->RemoveAllVehicles();	
}

//////////////////////////////////////////////////

OffsetPursuitDemo* OffsetPursuitDemo::Instance()
{
  static OffsetPursuitDemo instance;

  return &instance;
}


void OffsetPursuitDemo::Enter(AIWorld* pWorld)
{
	//add two units
	for(int i = 0; i<7; i++)
	{
		pWorld->AddVehicle();
	};

	//pWorld->Agents().at(0)->Steering()->ArriveOn();
	//pWorld->Agents().at(0)->Steering()->WanderOn();
	//pWorld->Agents().at(0)->SetMaxSpeed(100);
	pWorld->Agents().at(0)->Steering()->FollowPathOn();

	pWorld->Agents().at(1)->Steering()->OffsetEnterHelper(pWorld->Agents().at(0),Vector3(100.0,0.0,-100.0) );
	pWorld->Agents().at(1)->Steering()->OffsetPursuitOn( pWorld->Agents().at(0), Vector3(100.0,0.0,-100.0) );
	
	pWorld->Agents().at(2)->Steering()->OffsetEnterHelper(pWorld->Agents().at(0), Vector3(-100.0,0.0,-100.0));
	pWorld->Agents().at(2)->Steering()->OffsetPursuitOn( pWorld->Agents().at(0), Vector3(-100.0,0.0,-100.0) );

	pWorld->Agents().at(3)->Steering()->OffsetEnterHelper(pWorld->Agents().at(0),Vector3(150.0,0.0,-150.0) );
	pWorld->Agents().at(3)->Steering()->OffsetPursuitOn( pWorld->Agents().at(0), Vector3(150.0,0.0,-150.0) );

	pWorld->Agents().at(4)->Steering()->OffsetEnterHelper(pWorld->Agents().at(0), Vector3(-150.0,0.0,-150.0));
	pWorld->Agents().at(4)->Steering()->OffsetPursuitOn( pWorld->Agents().at(0), Vector3(-150.0,0.0,-150.0) );
	
	pWorld->Agents().at(5)->Steering()->OffsetEnterHelper(pWorld->Agents().at(0),Vector3(0.0,0.0,-50.0 ));
	pWorld->Agents().at(5)->Steering()->OffsetPursuitOn( pWorld->Agents().at(0), Vector3(0.0,0.0,-50.0) );

	pWorld->Agents().at(6)->Steering()->OffsetEnterHelper(pWorld->Agents().at(0),Vector3(0.0,0.0,-150.0 ));
	pWorld->Agents().at(6)->Steering()->OffsetPursuitOn( pWorld->Agents().at(0), Vector3(0.0,0.0,-150.0) );
}

void OffsetPursuitDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void OffsetPursuitDemo::Exit(AIWorld* pWorld)
{
  //turn off behaviours
	//pWorld->Agents().at(0)->Steering()->WanderOff();
	pWorld->Agents().at(0)->Steering()->FollowPathOn();
	//pWorld->Agents().at(0)->Steering()->ArriveOff();

	pWorld->Agents().at(1)->Steering()->OffsetExitHelper();
	pWorld->Agents().at(1)->Steering()->OffsetPursuitOff();

	pWorld->Agents().at(2)->Steering()->OffsetExitHelper();
	pWorld->Agents().at(2)->Steering()->OffsetPursuitOff();

	pWorld->Agents().at(3)->Steering()->OffsetExitHelper();
	pWorld->Agents().at(3)->Steering()->OffsetPursuitOff();

	pWorld->Agents().at(4)->Steering()->OffsetExitHelper();
	pWorld->Agents().at(4)->Steering()->OffsetPursuitOff();

	pWorld->Agents().at(5)->Steering()->OffsetExitHelper();
	pWorld->Agents().at(5)->Steering()->OffsetPursuitOff();

	pWorld->Agents().at(6)->Steering()->OffsetExitHelper();
	pWorld->Agents().at(6)->Steering()->OffsetPursuitOff();

	 //clear agent list
  pWorld->RemoveAllVehicles();	
}

//////////////////////////////////////////////////

InterposeDemo* InterposeDemo::Instance()
{
  static InterposeDemo instance;

  return &instance;
}


void InterposeDemo::Enter(AIWorld* pWorld)
{
	//add units
	for(int i = 0; i<3; i++)
	{
		pWorld->AddVehicle();
	};

   //turn on behaviours
	pWorld->Agents().at(0)->Steering()->WanderOn();
	pWorld->Agents().at(1)->Steering()->PursuitOn(pWorld->Agents().at(0));
	pWorld->Agents().at(2)->Steering()->InterposeOn(pWorld->Agents().at(0),pWorld->Agents().at(1));

}

void InterposeDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void InterposeDemo::Exit(AIWorld* pWorld)
{
    //turn off behaviours
	pWorld->Agents().at(0)->Steering()->WanderOff();
	pWorld->Agents().at(1)->Steering()->PursuitOff();
	pWorld->Agents().at(2)->Steering()->InterposeOff();

	 //clear agent list
  pWorld->RemoveAllVehicles();
}

//////////////////////////////////////////////////

FlockDemo* FlockDemo::Instance()
{
  static FlockDemo instance;

  return &instance;
}


void FlockDemo::Enter(AIWorld* pWorld)
{
	//TODO: i think flocking has a memory leak somewhere
	//probably with vehicle.cpp
	for(int i = 0; i<150; i++)
	{
		pWorld->AddVehicle();

		//FlockingOn also puts WanderOn()- dont want that atm
		//pWorld->Agents().at(i)->Steering()->FlockingOn();

		pWorld->Agents().at(i)->Steering()->SeparationOn();
		pWorld->Agents().at(i)->Steering()->AlignmentOn();
		pWorld->Agents().at(i)->Steering()->CohesionOn();
	}
  
}

void FlockDemo::Execute(AIWorld* pWorld)
{  
  
}


void FlockDemo::Exit(AIWorld* pWorld)
{
	for(int i = 0; i<150; i++)
	{
		pWorld->Agents().at(i)->Steering()->SeparationOff();
		pWorld->Agents().at(i)->Steering()->AlignmentOff();
		pWorld->Agents().at(i)->Steering()->CohesionOff();

		//pWorld->Agents().at(i)->Steering()->FlockingOff();
	}

  //clear agent list
  pWorld->RemoveAllVehicles();
}

//////////////////////////////////////////////////

FollowPathDemo* FollowPathDemo::Instance()
{
  static FollowPathDemo instance;

  return &instance;
}


void FollowPathDemo::Enter(AIWorld* pWorld)
{
 //add units
	for(int i = 0; i<2; i++)
	{
		pWorld->AddVehicle();
		pWorld->Agents().at(i)->Steering()->FollowPathOn();
	};
}

void FollowPathDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void FollowPathDemo::Exit(AIWorld* pWorld)
{
  for(int i = 0; i<2; i++)
	{
		pWorld->Agents().at(i)->Steering()->FollowPathOff();
	}

  //clear agent list
  pWorld->RemoveAllVehicles();
}

//////////////////////////////////////////////////

WanderDemo* WanderDemo::Instance()
{
  static WanderDemo instance;

  return &instance;
}


void WanderDemo::Enter(AIWorld* pWorld)
{
	pWorld->AddVehicle();
	pWorld->Agents().at(0)->Steering()->WanderOn();
}

void WanderDemo::Execute(AIWorld* pWorld)
{  
  //do nothing 
}


void WanderDemo::Exit(AIWorld* pWorld)
{
 pWorld->Agents().at(0)->Steering()->WanderOff();
  
  //clear agent list
  pWorld->RemoveAllVehicles();
}

//////////////////////////////////////////////////

ObstacleAvoidanceDemo* ObstacleAvoidanceDemo::Instance()
{
  static ObstacleAvoidanceDemo instance;

  return &instance;
}


void ObstacleAvoidanceDemo::Enter(AIWorld* pWorld)
{
	pWorld->CreateObstacles();

	pWorld->AddVehicle();
		
	pWorld->Agents().at(0)->Steering()->ObstacleAvoidanceOn();
	pWorld->Agents().at(0)->Steering()->ArriveOn();
	pWorld->Agents().at(0)->Steering()->ObstacleEnterHelper();
	
	
}

void ObstacleAvoidanceDemo::Execute(AIWorld* pWorld)
{  
}


void ObstacleAvoidanceDemo::Exit(AIWorld* pWorld)
{
  pWorld->Agents().at(0)->Steering()->ObstacleExitHelper();
  pWorld->Agents().at(0)->Steering()->ObstacleAvoidanceOff();
  pWorld->Agents().at(0)->Steering()->ArriveOff();

  pWorld->RemoveObstacles();

  //clear agent list
  pWorld->RemoveAllVehicles();
}