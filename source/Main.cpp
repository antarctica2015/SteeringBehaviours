
/*
	WinMain entry point
*/

#define WIN32_LEAN_AND_MEAN
#include <windows.h>

#include "OgreNewtonApplication.h"


INT WINAPI WinMain( HINSTANCE hInst, HINSTANCE, LPSTR strCmdLine, INT )
{
    // Create application object
    OgreNewtonApplication app;

    try {
        app.go();
    } catch( Ogre::Exception& e ) {
		printf( "An Error HAS OCCURED: %s", e.getFullDescription() );
    }

    return 0;
}