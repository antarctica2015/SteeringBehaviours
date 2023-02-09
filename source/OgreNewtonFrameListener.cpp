#include ".\ogrenewtonframelistener.h"

OgreNewtonFrameListener::OgreNewtonFrameListener(RenderWindow* win, Camera* cam, SceneManager* mgr, OgreNewt::World* W, SceneNode* ncam, AIWorld* aiWorld) :
		ExampleFrameListener(win,cam)
{
	m_World = W;
	msnCam = ncam;
	mSceneMgr = mgr;
	mAIWorld = aiWorld;

	//set up circle - could do with being improved
	createCircle(5);

	//setup 3D line node.
	mLineNode = mSceneMgr->getRootSceneNode()->createChildSceneNode();
	mLine = new ManualObject( "_LINE_" );

	//make a text overlay
	//Ogre::OverlayManager& overlayManager = Ogre::OverlayManager::getSingleton();
	
	// Create a panel
	//panel = static_cast<Ogre::OverlayContainer*>(Ogre::OverlayManager::getSingleton().createOverlayElement("Panel", "PanelName"));

	//panel->setMetricsMode(Ogre::GMM_PIXELS);
	//panel->setPosition(5, 5);
	//panel->setDimensions(400, 400);
	//panel->setColour(ColourValue(0.3, 0.5, 0.3));

	// Create a text area
	//textArea = static_cast<TextAreaOverlayElement*>(OverlayManager::getSingleton().createOverlayElement("TextArea", "TextAreaName"));
	//textArea->setMetricsMode(Ogre::GMM_PIXELS);
	//textArea->setPosition(10, 10);
	//textArea->setDimensions(190, 90);
	//textArea->setCharHeight(36);
	//textArea->setFontName("TrebuchetMSBold");
	//textArea->setColourBottom(ColourValue(0.3, 0.5, 0.3));
	//textArea->setColourTop(ColourValue(0.5, 0.7, 0.5));

	// Create an overlay, and add the panel
	//overlay = Ogre::OverlayManager::getSingleton().create("OverlayName");
	//overlay->add2D(panel);

	// Add the text area to the panel
	//panel->addChild(textArea);

}

OgreNewtonFrameListener::~OgreNewtonFrameListener(void)
{
}

bool OgreNewtonFrameListener::frameStarted(const FrameEvent &evt)
{
	mInputDevice->capture();

	// ----------------------------------------
	// CAMERA CONTROLS
	// ----------------------------------------

	//pan & scroll with arrow keys?
	//zoom in and out with + - or mouse wheel
	//mouse look when right button pressed

	
		Vector3 trans, strafe, vec, scroll;
		Quaternion quat;

		quat = msnCam->getOrientation();

		vec = Vector3(0.0,0.0,-0.5);
		trans = quat * vec;

		vec = Vector3(0.5,0.0,0.0);
		strafe = quat * vec;

		vec = Vector3(0.0,0.5,0.0);
		scroll = quat * vec;

	if( mInputDevice->getMouseButton(1) )
	{
		msnCam->pitch( Degree(mInputDevice->getMouseRelativeY() * -0.25) );
		msnCam->yaw( Degree(mInputDevice->getMouseRelativeX() * -0.25), SceneNode::TS_WORLD );

		//write additional cam code to rotate around a point in space instead of yaw
		//point could be gained from mouse cursor position or picking a unit
	}

		if( mInputDevice->getMouseButton(3) )
			msnCam->translate(trans);

		if( mInputDevice->getMouseButton(4) )
			msnCam->translate(trans * -1.0);

		//Keyboard camera controls
	
		if (mInputDevice->isKeyDown(Ogre::KC_UP))
			msnCam->translate(scroll);

		if (mInputDevice->isKeyDown(Ogre::KC_DOWN))
			msnCam->translate(scroll * -1.0);
		
		if (mInputDevice->isKeyDown(Ogre::KC_LEFT))
			msnCam->translate(strafe * -1.0);

		if (mInputDevice->isKeyDown(Ogre::KC_RIGHT))
			msnCam->translate(strafe);

		if (mInputDevice->isKeyDown(Ogre::KC_ADD))
			msnCam->translate(trans);

		if (mInputDevice->isKeyDown(Ogre::KC_SUBTRACT))
			msnCam->translate(trans * -1.0);

		////////////////////////////////////////////////////////
		//arrive demo controls
		if (((( (mAIWorld->current_state) == arrive)) || ( (mAIWorld->current_state) == seek) || 
				( (mAIWorld->current_state) == flee)  || ( (mAIWorld->current_state) == obstacle_avoidance)))
		{
			//TODO: a drawlineandcircle() type class would be good
			//and replace all this mess here
			Vector3 temp = Vector3(0.0,0.0,0.0);
			temp = mAIWorld->Crosshair();

			//draw a circle to show point
			mCircleNode->setVisible(true);
			mCircleNode->setPosition(temp);

			//also could do with drawing a line n units up along Y to clearly show point
			Ray lineRay = Ray(temp,Vector3::UNIT_Y);

			Ogre::Vector3 start, end;
			start = lineRay.getOrigin();
			end = lineRay.getPoint( 100.0 );

			//set up a line class?
			remove3DLine();
			mLine->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
			mLine->position( start );
			mLine->position( end );
			mLine->end();
			mLineNode->attachObject( mLine );
		
			if(mInputDevice->getMouseButton(0))
			{
				CEGUI::Window* win = CEGUI::System::getSingleton().getWindowContainingMouse();

				//If mouse if over a GUI element then dont raycast otherwise...
				if ((win == CEGUI::WindowManager::getSingleton().getWindow("root") ) || (win == NULL) )
				{
				Vector3 tempVec = Vector3(0.0,0.0,0.0) ;
				
				//perform a raycast
				CEGUI::Point mouse = CEGUI::MouseCursor::getSingleton().getPosition();
				CEGUI::Renderer* rend = CEGUI::System::getSingleton().getRenderer();

				Real mx,my;
				mx = mouse.d_x / rend->getWidth();
				my = mouse.d_y / rend->getHeight();

				//shoots a ray out from mouse pointer position into world
				Ray camray = mCamera->getCameraToViewportRay(mx,my);

				//create a plane to test against
				Ogre::Plane testPlane = Plane( Vector3::UNIT_Y, Vector3(0,0,0));
			
				//gets the distance along the ray where it intersects the given plane and resolves it to a point in space
				tempVec = camray.getPoint( camray.intersects(testPlane).second );
		
				//finally set this as the target point
				mAIWorld->SetCrosshair(tempVec);

				//draw a circle to show point
				mCircleNode->setVisible(true);
				mCircleNode->setPosition(tempVec);

				//also could do with drawing a line n units up along Y to clearly show point
				Ray lineRay = Ray(tempVec,Vector3::UNIT_Y);
		
				Ogre::Vector3 start, end;
				start = lineRay.getOrigin();
				end = lineRay.getPoint( 100.0 );

				//set up a line class?
				remove3DLine();
				mLine->begin("BaseWhiteNoLighting", Ogre::RenderOperation::OT_LINE_LIST );
				mLine->position( start );
				mLine->position( end );
				mLine->end();
				mLineNode->attachObject( mLine );
				}			
			}
			
	}
	else
	{
		//dont show line/circle
		remove3DLine();
		mCircleNode->setVisible(false);
	}

	//TODO: use overlay text or CEGUI?		
	//textArea->setCaption("Current state: " + StringConverter::toString(mAIWorld->current_state));
	//textArea->setCaption("Current state: " + StringConverter::toString(mAIWorld->icurrent));
	//overlay->show();

	if(mAIWorld->current_state == obstacle_avoidance)
	{
	mWindow->setDebugText(mAIWorld->Agents().at(0)->DebugText);
	}

	//Update mAIWorld and hence vehicles
	mAIWorld->Update(evt.timeSinceLastFrame);

	///////////////////////////////////////////////////////////////////////////////////////////
	//Quit key
	if (mInputDevice->isKeyDown(Ogre::KC_ESCAPE))
	{
		return false;
	}

	//TODO: print screen key

	return true;
}

void OgreNewtonFrameListener::remove3DLine()
{
	mLineNode->detachAllObjects();
	mLine->clear();	
}

//circle code taken from http://www.ogre3d.org/wiki/index.php/Circle
void OgreNewtonFrameListener::createCircle(double dRadius)
{
	// for 3d circle with thickness
	ManualObject * circle = mSceneMgr->createManualObject("circle");

    float const radius = dRadius,
                thickness = 2.0, // Of course this must be less than the radius value.
                accuracy = 5;

    //add in test to make sure thickness is less and if not make it so

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

	mCircleNode =  mSceneMgr->getRootSceneNode()->createChildSceneNode();
	mCircleNode->attachObject(circle);

	//dont draw it yet
	mCircleNode->setVisible(false);

}