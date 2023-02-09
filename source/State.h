#ifndef STATE_H
#define STATE_H
//------------------------------------------------------------------------
//
//  Name:   State.h
//
//  Desc:   abstract base class to define an interface for a state
//
//  Author: Mat Buckland (fup@ai-junkie.com)
//
//------------------------------------------------------------------------

class AIWorld;

class State
{
public:

  virtual ~State(){} //needs to have aiworld & scenemanger stuff, what else?...

  //this will execute when the state is entered
  virtual void Enter(AIWorld*)=0;

  //this is the states normal update function
  virtual void Execute(AIWorld*)=0;

  //this will execute when the state is exited. 
  virtual void Exit(AIWorld*)=0;

};

#endif