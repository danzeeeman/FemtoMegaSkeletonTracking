#pragma once

//--------------------------------------------------------------
//
// Femto Skeleton OSC
//
// Finds the people in front of one or more Orbbec Femto Megas that are on the network (Ethernet, PoE) and
// sends their skeletons out over OSC, in the form TrackerFemto sent them:
//
//     /bodies/<id>/joints/<Joint>   x y z "Tracked" | "NotTracked"
//
// Each camera runs RTMPose on its colour image and puts the joints in 3D with its depth image. Camera 0 is the
// reference: the skeletons go out in its frame, and the others are placed in it by a calibration.
//
// Threads: each camera has its own, which takes its newest frame, runs the pose models on it and builds that
// frame's skeletons from that frame's own depth. One more merges the cameras and sends the OSC as soon as any
// camera has new bodies. The app's own thread connects, draws and runs the window.
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
// Keys: space  send on / off   r  record every camera / stop
//
//--------------------------------------------------------------

#include "ofMain.h"
#include "ofxImGui.h"
#include "ofxOrbbecDevice.h"
#include "ofxOrbbecRecording.h"
#include "ofxOrbbecFinder.h"
#include "ofxRtmPoseTracker.h"
#include "ofxYoloPose.h"
#include "FemtoSkeleton.h"
#include "SkeletonFusion.h"
#include "Calibration.h"
#include <atomic>
#include <condition_variable>
#include <future>
#include <memory>
#include <mutex>
#include <set>
#include <thread>

class ofApp : public ofBaseApp {
public:
    // as many cameras as there can be: a slot costs nothing until its camera connects (then it gets a pose model and
    // a thread of its own)
    static const int kMaxCameras = 16;

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
        ofParameter< bool > invertImu;        // "Camera Upside Down": the accelerometer's up taken the other way (it reads gravity, not up)

        ofxOrbbecDevice device;
        bool fake = false;                      // FEMTO_DEMO: stands in for a camera
        // a recording played back in place of the camera (set on the app's thread with the worker stopped)
        std::shared_ptr< ofxOrbbecPlayer > player;
        ofxOrbbecRecorder recorder;             // the camera's frames to a .femto file, while recording
        string selectedTake;                    // the recording chosen in the panel
        bool userStopped = false;               // stopped with its button: not started again by the search (this session)
        bool active() const { return fake || device.isOpen() || ( player && player->isOpen() ); }
        std::shared_ptr< ofxRtmPoseTracker > tracker;   // set on the app's thread, under mtx; then only the worker runs it
        std::shared_ptr< ofxYoloPose > yolo;            // the same, for YOLO11-pose
        std::future< bool > connecting;
        float lastTry = -100.f;
        float lastTrackerTry = -100.f;          // the models failed to load: not again until a while later
        ofxOrbbecIntrinsics K;                  // set before the worker starts

        // its thread: the frames, the poses, the skeletons
        std::thread worker;
        std::atomic< bool > running{ false };
        std::atomic< size_t > framesIn{ 0 }, toTracker{ 0 };
        struct DebugCounts {
            int poses = 0, found = 0, scored = 0, withDepth = 0, built = 0, updates = 0;
            float scoreMax = 0.f, lastLog = 0.f;
            size_t framesAtLog = 0;
            std::set< uint64_t > ids;           // the tracker's ids seen this second
            int jointsMin = 99, jointsMax = 0;  // joints per body built
            int notBuilt = 0;                   // poses found but too few joints placed to send
        } dbg;

        // What the worker hands to the rest, and the calibration the GUI and the output thread share: under mtx.
        std::mutex mtx;
        ofPixels shown;                         // the newest colour picture, for the texture
        bool shownNew = false;
        struct Pose2D { uint64_t id = 0; vector< glm::vec2 > keypoints; vector< float > scores; ofRectangle box; bool skeleton = true; };
        vector< Pose2D > poses2d;               // the poses over the picture, colour pixels
        vector< FemtoSkeleton::Body > bodies;   // in this camera's (levelled) frame
        vector< FemtoSkeleton::Box > boxes;     // the people found whose skeletons could not be built, as boxes
        double bodiesTime = 0.0;
        bool leveled = false;                   // the accelerometer levelled the last bodies
        glm::mat3 levelM{ 1.f };                // camera frame -> levelled frame, when `leveled`
        float pitchDeg = 0.f, rollDeg = 0.f;    // what the accelerometer says
        ofTexture tex;                          // the app's thread only

        // where it is relative to camera 0
        glm::mat4 toRef{ 1.f };
        bool calibrated = false;
        float calRms = 0.f;
        int calInliers = 0;
        bool collecting = false;
        double calTakenAt = -1.0;               // bodiesTime of the last pair added to `cal`, so a pair is added once
        Calibration cal;
        string calMessage;
        float forgetAskedAt = -100.f;   // "Forget calibration" pressed once: asking to be sure
    };

    void setupCamera( Camera& aCam );
    void setupTracker( Camera& aCam );
    void connectCamera( Camera& aCam );
    void startCamera( Camera& aCam );
    void playTake( Camera& aCam, const string& aPath );
    void stopTake( Camera& aCam );
    bool recordingAny() const;
    void toggleRecording();                 // every connected camera starts recording, or all stop
    void listTakes();
    vector< string > mTakes;                // the .femto recordings in bin/data/femto_takes, newest first
    float mTakesListedAt = -100.f;
    float mSettingsSavedAt = -100.f;   // for the "saved" note
    void stopCamera( Camera& aCam );
    void cameraLoop( Camera& aCam );
    void outputLoop();
    void wakeOutput();
    void collectCalibration( Camera& aCam, const vector< FemtoSkeleton::Body >& aRefBodies, double aRefTime );
    void drawCameraPanel( Camera& aCam );
    void drawSettingsWindow( const ofRectangle& aArea );
    void drawCamerasWindow( const ofRectangle& aArea );
    void startCameraByUser( Camera& aCam );
    void stopCameraByUser( Camera& aCam );

    // every Femto on the network (and USB), looked for every few seconds; those found are started
    ofxOrbbecFinder mFinder;
    void adoptFound();
    float mAdoptedAt = -100.f;
    char mAddIp[64] = "";
    void applyOsc();
    void updateDemo();
    void draw3d( const ofRectangle& aView, bool aTopDown );
    void floorFromFeet();
    void loadSettings();
    void saveSettings();
    void loadCalibration();
    void saveCalibration();

    ofxImGui::Gui gui;
    ofxImGui::Settings guiSettings;

    std::vector< std::unique_ptr< Camera > > mCams;
    FemtoSkeleton mSkeleton;                // its settings are read by every thread; its sender is under mOscMutex
    SkeletonFusion mFusion;                 // under mFusionMutex
    std::mutex mOscMutex, mFusionMutex;

    // the thread that merges the cameras and sends: woken by a camera with new bodies
    std::thread mOutThread;
    std::atomic< bool > mOutRunning{ false };
    std::mutex mOutWakeMutex;
    std::condition_variable mOutCv;
    bool mOutWake = false;

    ofParameterGroup params, mAllParams;
    ofParameter< string > oscHost;
    ofParameter< int > oscPort;
    ofParameter< bool > bSend;
    ofParameter< int > cameraPort;
    ofParameter< int > colorWidth, colorHeight;
    ofParameter< bool > useYolo;            // the pose model: YOLO11n-pose (people and 17 keypoints in one pass), else RTMW

    // The joined space in 3D: an orbit camera, Z up. Display axes are x right, y away from camera 0, z up.
    ofCamera mView;
    float mViewAz = -110.f, mViewEl = 22.f, mViewDist = 6.f;
    glm::vec3 mViewTarget{ 0.f, 2.5f, -0.8f };
    ofRectangle mViewRect;
    glm::vec2 mTopCentre{ 0.f, 3.f };  // the top-down view's framing (display x, y), eased toward what it must take in
    float mTopHalf = 0.f;
    int mLastMx = 0, mLastMy = 0;
    ofParameter< float > floorBelow;   // how far the floor is below camera 0 (m)

    bool mDemo = false;
    bool mDebug = false;               // debug builds only: once a second, where the skeletons are lost
    float mAutoQuitSec = 0.f;
    std::atomic< size_t > mBodiesSent{ 0 };
};
