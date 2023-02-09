#ifndef _CEGUIFRAMELISTENER_
#define _CEGUIFRAMELISTENER_

// CEGUI memory manager issues...
#include <OgreNoMemoryMacros.h>
#include <CEGUI/CEGUI.h>
#include <OgreMemoryMacros.h>

#include <OgreCEGUIRenderer.h>
#include <OgreCEGUIResourceProvider.h>

#include "ExampleFrameListener.h"
#include <OgreEventQueue.h>


///////////////////////////////////////////////////////////////////////////////////////////////////////


class CEGUIFrameListener : public ExampleFrameListener, Ogre::MouseMotionListener, Ogre::MouseListener
{
public:
	CEGUIFrameListener(RenderWindow* win, Ogre::Camera* cam) : ExampleFrameListener(win,cam)
	{
		mInputDevice2 = mInputDevice;

		mEventProcessor = new Ogre::EventProcessor();
		mEventProcessor->initialise(win);
		mEventProcessor->startProcessingEvents();
		mEventProcessor->addKeyListener(this);
		mEventProcessor->addMouseMotionListener(this);
		mEventProcessor->addMouseListener(this);
		mInputDevice = mEventProcessor->getInputReader();

	}

	~CEGUIFrameListener()
	{
		mEventProcessor->stopProcessingEvents();
		mEventProcessor->removeKeyListener(this);
		mEventProcessor->removeMouseMotionListener(this);
		mEventProcessor->removeMouseListener(this);
		delete mEventProcessor;

		mInputDevice = mInputDevice2;
	}


	virtual void 	mouseMoved (MouseEvent *e)
	{
		CEGUI::Renderer* rend = CEGUI::System::getSingleton().getRenderer();
		CEGUI::System::getSingleton().injectMouseMove(e->getRelX() * rend->getWidth(), e->getRelY() * rend->getHeight());
		e->consume();
	}


	virtual void 	mouseDragged (MouseEvent *e)
	{
		mouseMoved(e);
	}


	virtual void 	keyPressed (KeyEvent *e)
	{

		// do event injection
		CEGUI::System& cegui = CEGUI::System::getSingleton();

		// key down
		cegui.injectKeyDown(e->getKey());

		// now character
		cegui.injectChar(e->getKeyChar());

		e->consume();
	}


	virtual void 	keyReleased (KeyEvent *e)
	{
		CEGUI::System::getSingleton().injectKeyUp(e->getKey());
	}



	virtual void 	mousePressed (MouseEvent *e)
	{
		CEGUI::System::getSingleton().injectMouseButtonDown(convertOgreButtonToCegui(e->getButtonID()));
		e->consume();
	}


	virtual void 	mouseReleased (MouseEvent *e)
	{
		CEGUI::System::getSingleton().injectMouseButtonUp(convertOgreButtonToCegui(e->getButtonID()));
		e->consume();
	}


	virtual void    mouseWheelEvent(MouseEvent *e)
	{
	//using namespace CEGUI;
	//mTranslateVector.z += ((const MouseEventArgs&)e).wheelChange * -5.0;
	CEGUI::System::getSingleton().injectMouseWheelChange(convertOgreButtonToCegui(e->getButtonID()));
	e->consume();
	}

	virtual void handleMouseWheelEvent(const CEGUI::EventArgs& e) 
    { 
        using namespace CEGUI; 
        mTranslateVector.z += ((const MouseEventArgs&)e).wheelChange * -5.0;  
    } 
	
	// do-nothing events
	virtual void 	keyClicked (KeyEvent *e) {}
	virtual void 	mouseClicked (MouseEvent *e) {}
	virtual void 	mouseEntered (MouseEvent *e) {}
	virtual void 	mouseExited (MouseEvent *e) {}


	bool frameStarted(const FrameEvent& evt);

protected:
	CEGUI::MouseButton convertOgreButtonToCegui(int ogre_button_id)
	{
		switch (ogre_button_id)
		{
		case MouseEvent::BUTTON0_MASK:
			return CEGUI::LeftButton;
			break;

		case MouseEvent::BUTTON1_MASK:
			return CEGUI::RightButton;
			break;

		case MouseEvent::BUTTON2_MASK:
			return CEGUI::MiddleButton;
			break;

		case MouseEvent::BUTTON3_MASK:
			return CEGUI::X1Button;
			break;

		default:
			return CEGUI::LeftButton;
			break;
		}

	}

protected:
	Ogre::EventProcessor* mEventProcessor;
	Ogre::InputReader* mInputDevice2;

};




#endif	