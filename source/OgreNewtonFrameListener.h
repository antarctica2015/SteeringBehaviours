#pragma once
#include "ExampleFrameListener.h"
#include <OgreNewt.h>
#include <OgreTextAreaOverlayElement.h>
#include "AIWorld.h"

//#include "OgreNewtonApplication.h"

#include "CEGUIFrameListener.h"

class OgreNewtonFrameListener :
	public ExampleFrameListener
{
protected:
	OgreNewt::World*	m_World;
	SceneNode*			msnCam;
	SceneManager*		mSceneMgr;

	AIWorld*            mAIWorld;

	// for drawing 3D lines & circle
	SceneNode* mCircleNode;
	SceneNode* mLineNode;
	ManualObject* mLine;

	void createCircle(double dRadius);
	void remove3DLine();

	int count;
	float timer;

public:
	
	OgreNewtonFrameListener(RenderWindow* win, Camera* cam, SceneManager* mgr, OgreNewt::World* W, SceneNode* ncam, AIWorld* mAIWorld);
	~OgreNewtonFrameListener(void);

	bool frameStarted(const FrameEvent &evt);

	//to display text
	//Ogre::OverlayContainer* panel;
	//TextAreaOverlayElement* textArea;
	//Ogre::Overlay*			overlay;

};
