#pragma once

//--------------------------------------------------------------
//
// Femto Skeleton OSC
//
// Finds the people in front of one or more Orbbec Femto Megas that are on the network (Ethernet, PoE) and
// sends their skeletons out over OSC, in the form TrackerFemto sent them and Echo & Narcissus listens for:
//
//     /bodies/<id>/joints/<Joint>   x y z "Tracked" | "NotTracked"
//
// Each camera runs RTMPose on its colour image and puts the joints in 3D with its depth image. Camera 0 is the
// reference: the skeletons go out in its frame, and the others are placed in it by a calibration.
//
// Calibration, per camera, with a person as the target: tick "Collect points" and walk about where the camera
// and camera 0 can both see you (alone in view). Each Femto's accelerometer gives its pitch and roll, so what is
// left to find is a turn about the vertical and a shift, which a few seconds of walking pin down; "Solve" finds
// them and shows how well they fit. The result is kept in bin/data/calibration.json.
//
// Where a person is seen by several cameras, the skeletons are merged into one with one id.
//
// Each Femto's address is set in the window (a Femto Mega's factory address is 192.168.1.10; give each its own,
// and the computer an address in the same subnet). Settings are kept in bin/data/settings.xml. The pose models
// are downloaded on the first run into bin/data/models.
//
// Keys: space  send on / off
//
//--------------------------------------------------------------

#include "ofMain.h"
#include "ofxImGui.h"
#include "ofxOrbbecDevice.h"
#include "ofxRtmPoseTracker.h"
#include "FemtoSkeleton.h"
#include "SkeletonFusion.h"
#include "Calibration.h"
#include <future>
#include <memory>

class ofApp : public ofBaseApp {
public:
    static const int kMaxCameras = 4;

    void setup() override;
    void update() override;
    void draw() override;
    void exit() override;
    void keyPressed( int key ) override;
    void mousePressed( int x, int y, int button ) override;
    void mouseDragged( int x, int y, int button ) override;
    void mouseScrolled( int x, int y, float scrollX, float scrollY ) override;

private:
    // One Femto Mega and what is known about it.
    struct Camera {
        int index = 0;
        ofParameterGroup params;
        ofParameter< bool > enabled;
        ofParameter< string > ip;
        ofParameter< bool > levelFromImu;     // pitch and roll from the accelerometer, not the sliders
        ofParameter< bool > invertImu;        // the accelerometer reads gravity, not up

        ofxOrbbecDevice device;
        bool fake = false;                      // FEMTO_DEMO: stands in for a camera
        bool active() const { return fake || device.isOpen(); }
        std::shared_ptr< ofxRtmPoseTracker > tracker;
        std::future< bool > connecting;
        float lastTry = -100.f;
        float lastTrackerTry = -100.f;          // the models failed to load: not again until a while later

        ofPixels color;
        ofShortPixels depth;
        ofTexture tex;
        ofxOrbbecIntrinsics K;
        size_t framesIn = 0, toTracker = 0;

        vector< FemtoSkeleton::Body > bodies;   // in this camera's (levelled) frame
        double bodiesTime = 0.0;
        bool newBodies = false;                 // the tracker produced something this update
        bool leveled = false;                   // the accelerometer levelled the last bodies
        glm::mat3 levelM{ 1.f };                // camera frame -> levelled frame, when `leveled`
        float pitchDeg = 0.f, rollDeg = 0.f;    // what the accelerometer says

        // where it is relative to camera 0
        glm::mat4 toRef{ 1.f };
        bool calibrated = false;
        float calRms = 0.f;
        int calInliers = 0;
        bool collecting = false;
        Calibration cal;
        string calMessage;
    };

    void setupCamera( Camera& aCam );
    void setupTracker( Camera& aCam );
    void connectCamera( Camera& aCam );
    void updateCamera( Camera& aCam );
    void collectCalibration( Camera& aCam );
    void drawCameraPanel( Camera& aCam );
    void applyOsc();
    void updateDemo();
    void draw3d( const ofRectangle& aView );
    void floorFromFeet();
    void loadSettings();
    void saveSettings();
    void loadCalibration();
    void saveCalibration();

    ofxImGui::Gui gui;
    ofxImGui::Settings guiSettings;

    std::vector< std::unique_ptr< Camera > > mCams;
    FemtoSkeleton mSkeleton;
    SkeletonFusion mFusion;

    ofParameterGroup params, mAllParams;
    ofParameter< string > oscHost;
    ofParameter< int > oscPort;
    ofParameter< bool > bSend;
    ofParameter< int > cameraPort;
    ofParameter< int > colorWidth, colorHeight;

    // The joined space in 3D: an orbit camera, Z up. Display axes are x right, y away from camera 0, z up.
    ofCamera mView;
    float mViewAz = -110.f, mViewEl = 22.f, mViewDist = 6.f;
    glm::vec3 mViewTarget{ 0.f, 2.5f, -0.8f };
    ofRectangle mViewRect;
    int mLastMx = 0, mLastMy = 0;
    ofParameter< float > floorBelow;   // how far the floor is below camera 0 (m)

    bool mDemo = false;
    bool mDebug = false;               // debug builds only: once a second, where the skeletons are lost
    float mAutoQuitSec = 0.f;
    size_t mBodiesSent = 0;
};
