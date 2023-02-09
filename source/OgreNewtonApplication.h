#pragma once

#include <ExampleApplication.h>
#include <OgreNewt.h>
#include ".\OgreNewtonFrameListener.h"
#include "AIWorld.h"

//used to make an list item in the menu combobox
class MyListItem : public CEGUI::ListboxTextItem
{
public:
    MyListItem(const CEGUI::String& text, CEGUI::uint id) : CEGUI::ListboxTextItem(text, id)
	{
		setSelectionBrushImage("TaharezLook", "MultiListSelectionBrush");	
	}
};


//main application class

class OgreNewtonApplication :
	public ExampleApplication
{
public:
	OgreNewtonApplication(void);
	~OgreNewtonApplication(void);

protected:
	void createFrameListener();
	void createScene();
	void destroyScene();

public:
	
	//Required
	SceneNode*				   msnCam;
	FrameListener*	           mNewtonListener;

	//
	OgreNewt::World*		   m_World; //physics manager
	AIWorld*				   mAIWorld; // ai manager
	float					   timer;

	//GUI
	CEGUI::OgreCEGUIRenderer*  mGUIRenderer;
	CEGUIFrameListener*		   mGUIFrameListener;
	CEGUI::System*             mGUISystem;

	
	void initComboBoxes(void);
	void initDemoEventWiring(void);
	bool handleDemoComboChanged(const CEGUI::EventArgs& e);
	
};

