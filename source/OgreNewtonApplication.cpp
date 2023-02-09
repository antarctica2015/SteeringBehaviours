#include ".\OgreNewtonApplication.h"
#include <OgreNewt.h>

OgreNewtonApplication::OgreNewtonApplication(void)
{
	// create OgreNewt world.
	m_World = new OgreNewt::World();
	
	//set size of physics world
	//m_World->setWorldSize(Ogre::Vector3(-2000.0,-100.0,-2000.0) , Ogre::Vector3(2000.0,100.0,2000.0));

}

OgreNewtonApplication::~OgreNewtonApplication(void)
{
	// destroy world.
	delete m_World;

	OgreNewt::Debugger::getSingleton().deInit();
}

void OgreNewtonApplication::createScene()
{
	// setup CEGUI
	mGUIRenderer = new CEGUI::OgreCEGUIRenderer( mWindow, Ogre::RENDER_QUEUE_OVERLAY, false, 3000, mSceneMgr );
	new CEGUI::System( mGUIRenderer );

	// load up CEGUI font and skin
	try
	{
		using namespace CEGUI;
		CEGUI::Logger::getSingleton().setLoggingLevel( CEGUI::Informative );

		CEGUI::SchemeManager::getSingleton().loadScheme((CEGUI::utf8*)"../../Media/GUI/schemes/WindowsLook.scheme");
		CEGUI::SchemeManager::getSingleton().loadScheme((CEGUI::utf8*)"../../Media/GUI/schemes/TaharezLookSkin.scheme");
	
		//set up root window & default behaviours
		CEGUI::Window* sheet = CEGUI::WindowManager::getSingleton().createWindow( (CEGUI::utf8*)"DefaultWindow", (CEGUI::utf8*)"root" );
		CEGUI::System::getSingleton().setGUISheet( sheet );
		CEGUI::System::getSingleton().setDefaultMouseCursor((CEGUI::utf8*)"WindowsLook", (CEGUI::utf8*)"MouseArrow");
		CEGUI::System::getSingleton().setDefaultFont((CEGUI::utf8*)"BlueHighway-12"); //this font will do for now
		
		//set up demo type menu
		Window* cBox = WindowManager::getSingleton().createWindow("TaharezLook/Combobox", "DemoCombo");
		sheet->addChildWindow(cBox);
		
		//window properties
		cBox->setPosition( Point( 0.80f, 0.0f ) );
		cBox->setSize( Size( 0.2f, 0.4f ) );
		cBox->setInheritsAlpha(false);
		cBox->setAlpha(0.75);
		cBox->setText("Select Demo");
		
		//set up camera type menu
		//Window* camBox = WindowManager::getSingleton().createWindow("TaharezLook/Combobox", "CamCombo");
		//sheet->addChildWindow(camBox);
		//window properties
		//camBox->setPosition( Point( 0.0f, 0.0f ) );
		//camBox->setSize( Size( 0.15f, 0.25f ) );
		//camBox->setInheritsAlpha(false);
		//camBox->setText("Select Camera");

		//setup funcs
		initComboBoxes();
		initDemoEventWiring();
		
	}
	catch (CEGUI::Exception)
	{}

	// Set up A.I. world
	mAIWorld = new AIWorld(mSceneMgr);

	// sky box.
	mSceneMgr->setSkyBox(true, "Examples/SpaceSkyBox");
	
	// need ambient light
	mSceneMgr->setAmbientLight( ColourValue( 1.0, 1.0, 1.0 ) );

	// position camera
	msnCam = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	msnCam->attachObject( mCamera );
	mCamera->setPosition(0.0, 0.0, 0.0);
	msnCam->setPosition( 0.0, 2500.0, 0.0);

	//camera defaults to looking along -Z, make it look down along -Y (by looking at a point in this case)
	msnCam->lookAt(Vector3(0,0,0),msnCam->TS_WORLD, Vector3::NEGATIVE_UNIT_Z);

}

void OgreNewtonApplication::createFrameListener()
{
	//key listener
	mFrameListener = new OgreNewtonFrameListener( mWindow, mCamera, mSceneMgr, m_World, msnCam, mAIWorld );
	mRoot->addFrameListener(mFrameListener);

	//physics listener
	mNewtonListener = new OgreNewt::BasicFrameListener( mWindow, mSceneMgr, m_World, 60 );
	mRoot->addFrameListener(mNewtonListener);

	//gui/mouse listener
	//TODO: create an expanded version of this could replace the standard frame listener
	mGUIFrameListener = new CEGUIFrameListener( mWindow, mCamera );
}

void OgreNewtonApplication::destroyScene()
{
	// CEGUI Cleanup
	CEGUI::System* sys = CEGUI::System::getSingletonPtr();
	delete sys;
	delete mGUIRenderer;

}

void OgreNewtonApplication::initComboBoxes(void)
{
	using namespace CEGUI;

	//demo select box
	Combobox* cbobox = (Combobox*)WindowManager::getSingleton().getWindow("DemoCombo");
	cbobox->setReadOnly(true);
    
	cbobox->addItem(new MyListItem( "None"           ,0) );
	cbobox->addItem(new MyListItem( "Seek"           ,1) );
	cbobox->addItem(new MyListItem( "Flee"           ,2) );
	cbobox->addItem(new MyListItem( "Arrive"         ,3) );
	cbobox->addItem(new MyListItem( "Evade"          ,4) );
	cbobox->addItem(new MyListItem( "Pursuit"        ,5) );
	cbobox->addItem(new MyListItem( "Offset Pursuit" ,6) );
	cbobox->addItem(new MyListItem( "Interpose"      ,7) );
	cbobox->addItem(new MyListItem( "Flock"          ,8) );
	cbobox->addItem(new MyListItem( "Follow Path"    ,9) );
	cbobox->addItem(new MyListItem( "Wander"         ,10) );
	cbobox->addItem(new MyListItem( "Obstacle Avoidance" ,11) );

	//camera select box
	//cbobox = (Combobox*)WindowManager::getSingleton().getWindow("CamCombo");
	//cbobox->setReadOnly(true);
    
	//cbobox->addItem(new MyListItem( "Free"   ,0) );
	//cbobox->addItem(new MyListItem( "Chase"   ,1) );
	//cbobox->addItem(new MyListItem( "Track"   ,2) );
}

void OgreNewtonApplication::initDemoEventWiring(void)
{
	using namespace CEGUI;

	//demo box
	WindowManager::getSingleton().getWindow("DemoCombo")->subscribeEvent(Combobox::EventListSelectionAccepted, CEGUI::Event::Subscriber(&OgreNewtonApplication::handleDemoComboChanged, this));
	
	//camera box - not used yet
	//WindowManager::getSingleton().getWindow("CamCombo")->subscribeEvent(Combobox::EventListSelectionAccepted, CEGUI::Event::Subscriber(&OgreNewtonApplication::handleDemoComboChanged, this))
}

bool OgreNewtonApplication::handleDemoComboChanged(const CEGUI::EventArgs& e)
{
	// get the selected mesh filename from the combo box
	CEGUI::ListboxItem* item = ((CEGUI::Combobox*)((const CEGUI::WindowEventArgs&)e).window)->getSelectedItem();
   
	//get menu item id as an enum
	demo_state_type	demo_type = (demo_state_type)item->getID();
	
	//pass in in to state selector
	mAIWorld->SelectState(demo_type);

	return true;
}
