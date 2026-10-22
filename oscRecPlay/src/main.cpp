#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
    ofGLFWWindowSettings settings;
    settings.setSize( 640, 520 );
    settings.title = "OSC Record Play";
    auto window = ofCreateWindow( settings );
    ofRunApp( window, std::make_shared< ofApp >() );
    ofRunMainLoop();
}
