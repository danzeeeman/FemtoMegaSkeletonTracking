#include "ofMain.h"
#include "ofApp.h"

//========================================================================
int main( ){
    ofGLFWWindowSettings settings;
    settings.setSize( 1500, 800 );
    settings.title = "Femto Skeleton OSC";
    auto window = ofCreateWindow( settings );
    ofRunApp( window, std::make_shared< ofApp >() );
    ofRunMainLoop();
}
