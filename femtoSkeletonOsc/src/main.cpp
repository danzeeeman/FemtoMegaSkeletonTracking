#include "ofMain.h"
#include "ofApp.h"

// --window x y w h: where to open and how big (Echo and Narcissus Controller App lays the apps out on the screen with it)
static bool windowArg( int argc, char** argv, ofGLFWWindowSettings& s ) {
    for( int i = 1; i + 4 < argc; ++i ) {
        if( std::string( argv[i] ) != "--window" ) continue;
        s.setPosition( glm::vec2( std::atof( argv[i + 1] ), std::atof( argv[i + 2] )));
        s.setSize( std::atoi( argv[i + 3] ), std::atoi( argv[i + 4] ));
        return true;
    }
    return false;
}

//========================================================================
int main( int argc, char** argv ){
    ofGLFWWindowSettings settings;
    settings.setSize( 1500, 800 );
    settings.title = "Femto Skeleton OSC";
    windowArg( argc, argv, settings );
    auto window = ofCreateWindow( settings );
    ofRunApp( window, std::make_shared< ofApp >() );
    ofRunMainLoop();
}
