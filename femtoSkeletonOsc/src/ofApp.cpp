#include "ofApp.h"
#include "OperatorTheme.h"
#include <unistd.h>
#include "ofxRtmModelLoader.h"
#include <random>
#include <glm/gtc/matrix_transform.hpp>

namespace {
// each camera's colour: OperatorTheme.h

// The matrix that turns a point in the camera's frame into the levelled frame (y down along gravity, z the way the
// camera faces, flattened onto the horizon), from the unit vector `aUp` (which way is up, camera frame).
bool levelMatrix( const glm::vec3& aUp, glm::mat3& aOut ) {
    const glm::vec3 ty = -aUp;
    glm::vec3 tz = glm::vec3( 0.f, 0.f, 1.f ) - ty * ty.z;   // the camera's forward, with the part along gravity taken out
    if( glm::length( tz ) < 0.2f ) return false;             // looking nearly straight up or down
    tz = glm::normalize( tz );
    const glm::vec3 tx = glm::normalize( glm::cross( ty, tz ));
    aOut = glm::transpose( glm::mat3( tx, ty, tz ));
    return true;
}

glm::vec3 vec3Of( const ofVec3f& v ) { return glm::vec3( v.x, v.y, v.z ); }

// A group's parameters without a header of their own (ofxImGui::AddGroup adds a tree node, and has no case for
// strings, so an address could not be typed in). On / off settings are toggle buttons, groups within are trees.
void addParams( ofParameterGroup& aGroup ) {
    for( auto& p : aGroup ) {
        if( auto tg = std::dynamic_pointer_cast< ofParameterGroup >( p )) {
            if( ImGui::TreeNode( tg->getName().c_str() )) { addParams( *tg ); ImGui::TreePop(); }
        }
        else if( auto ts = std::dynamic_pointer_cast< ofParameter< string > >( p )) ofxImGui::AddParameter( *ts );
        else if( auto tb = std::dynamic_pointer_cast< ofParameter< bool > >( p )) optheme::toggle( *tb );
        else if( auto ti = std::dynamic_pointer_cast< ofParameter< int > >( p )) {
            // a port is typed in, not dragged to
            if( ofIsStringInString( ofToLower( ti->getName() ), "port" )) {
                int tv = ti->get();
                if( ImGui::InputInt( ti->getName().c_str(), &tv, 0, 0 )) ti->set( ofClamp( tv, ti->getMin(), ti->getMax() ));
            } else {
                ofxImGui::AddParameter( *ti );
            }
        }
        else if( auto tf = std::dynamic_pointer_cast< ofParameter< float > >( p )) ofxImGui::AddParameter( *tf );
    }
}

// FEMTO_SELFTEST=1: no cameras needed. Two virtual cameras watch a person walk about; camera 1 is placed with a
// known pose, with joint noise. Checks that calibration finds the pose (levelled and not) and that fusion joins
// the two views into one person.
void runSelfTest() {
    std::mt19937 trng( 7 );
    std::normal_distribution< float > tnoise( 0.f, 0.015f );
    // camera 1's frame -> camera 0's: a turn of 40 degrees about the vertical and a shift
    const glm::mat4 ttrue = glm::translate( glm::mat4( 1.f ), glm::vec3( 1.8f, 0.05f, 2.4f )) * glm::rotate( glm::mat4( 1.f ), glm::radians( 40.f ), glm::vec3( 0.f, 1.f, 0.f ));
    const glm::mat4 tinv = glm::inverse( ttrue );
    Calibration tcalYaw, tcalFull;
    for( int step = 0; step < 400; step++ ) {
        // the person walks a loop in front of camera 0 (floor y = 1.5 m below it), arms and legs at fixed offsets
        const float ta = step * 0.05f;
        const glm::vec3 tbase( 1.0f * std::cos( ta ), 1.5f, 2.0f + 0.9f * std::sin( ta * 0.8f ));
        for( int j = 0; j < 12; j++ ) {
            const glm::vec3 tjoint = tbase + glm::vec3( 0.2f * std::sin( j * 1.3f ), -1.7f * ( j / 11.f ), 0.15f * std::cos( j * 2.1f ));
            glm::vec3 ta0 = tjoint + glm::vec3( tnoise( trng ), tnoise( trng ), tnoise( trng ));
            glm::vec3 tb1 = glm::vec3( tinv * glm::vec4( tjoint, 1.f )) + glm::vec3( tnoise( trng ), tnoise( trng ), tnoise( trng ));
            tcalYaw.add( tb1, ta0 );
            tcalFull.add( tb1, ta0 );
        }
    }
    for( int mode = 0; mode < 2; mode++ ) {
        Calibration::Result tr = ( mode == 0 ? tcalYaw : tcalFull ).solve( mode == 0 );
        float terr = 0.f;
        for( int c = 0; c < 4; c++ ) for( int r = 0; r < 4; r++ ) terr = std::max( terr, std::abs( tr.toRef[c][r] - ttrue[c][r] ));
        ofLogNotice( "selftest" ) << ( mode == 0 ? "yaw+shift: " : "full 3D:   " ) << ( tr.ok ? "ok" : tr.message ) << ", rms " << tr.rms * 100.f
                                  << " cm, inliers " << tr.inliers << "/" << tr.total << ", largest matrix error " << terr;
    }

    // fusion: the same person seen by two cameras, then a second person
    SkeletonFusion tfusion;
    auto body = [&]( const glm::vec3& c, uint64_t id ) {
        FemtoSkeleton::Body b;
        b.id = id;
        for( int j : { (int)FemtoSkeleton::SPINE_MID, (int)FemtoSkeleton::HEAD, (int)FemtoSkeleton::HAND_LEFT } ) {
            glm::vec3 p = c + glm::vec3( 0.f, -0.2f * j * 0.1f, 0.f ) + glm::vec3( tnoise( trng ), tnoise( trng ), tnoise( trng ));
            b.cam[j] = ofVec3f( p.x, p.y, p.z );
            b.seen[j] = true;
        }
        return b;
    };
    uint64_t tfirst = 0;
    bool tstable = true;
    for( int f = 0; f < 50; f++ ) {
        std::vector< SkeletonFusion::Detection > td;
        glm::vec3 tp( 0.01f * f, 0.f, 2.f );
        td.push_back( { 0, body( tp, 11 ) });
        td.push_back( { 1, body( tp, 99 ) });
        if( f > 25 ) td.push_back( { 0, body( glm::vec3( -1.5f, 0.f, 3.f ), 12 ) });
        auto& tt = tfusion.update( td, f * 0.03 );
        if( f == 0 ) tfirst = tt[0].id;
        if( tt[0].id != tfirst ) tstable = false;
    }
    int tseen = 0;
    for( auto& t : tfusion.tracks() ) if( t.updated ) tseen++;
    ofLogNotice( "selftest" ) << "fusion: " << tseen << " people (want 2), the first kept its id: " << ( tstable ? "yes" : "NO" )
                              << ", cameras in it: " << tfusion.tracks()[0].cameras;
}

}

//--------------------------------------------------------------
void ofApp::setup() {
    // started from Finder there is no terminal to see the log in: it goes to bin/data/femtoSkeletonOsc.log
    if( !isatty( STDERR_FILENO )) ofLogToFile( "femtoSkeletonOsc.log", false );
    if( ofGetEnv( "FEMTO_SELFTEST" ) != "" ) {
        runSelfTest();
        ofExit( 0 );
        return;
    }
    // plain 2D textures: ImGui draws them as GL_TEXTURE_2D
    ofDisableArbTex();
    ofSetFrameRate( 60 );
    ofSetVerticalSync( false );
    ofBackground( optheme::bg );

    gui.setup();
    optheme::applyImGui();

    params.setName( "Output" );
    params.add( oscHost.set( "OSC host", "127.0.0.1" ));
    params.add( oscPort.set( "OSC port", 9999, 1, 65535 ));
    params.add( bSend.set( "Send skeletons", true ));
    params.add( cameraPort.set( "Camera port (all)", 8090, 1, 65535 ));
    // 0 = the camera's fastest mode (highest frame rate, then the smallest image)
    params.add( colorWidth.set( "Colour width (0 fastest)", 0, 0, 3840 ));
    params.add( colorHeight.set( "Colour height (0 fastest)", 0, 0, 2160 ));
    params.add( floorBelow.set( "Floor below camera 0 (m)", 1.5f, 0.f, 5.f ));
    params.add( useYolo.set( "Pose model YOLO11n-pose (else RTMW)", true ));

    // everything that is kept goes in and out of one file
    mAllParams.setName( "FemtoSkeletonOsc" );
    mAllParams.add( params );
    mAllParams.add( mSkeleton.getParams() );
    mAllParams.add( mFusion.params );
    for( int i = 0; i < kMaxCameras; i++ ) {
        auto tcam = std::make_unique< Camera >();
        tcam->index = i;
        setupCamera( *tcam );
        mAllParams.add( tcam->params );
        mCams.push_back( std::move( tcam ));
    }
    loadSettings();
    loadCalibration();
    applyOsc();

    // ofToFloat("") leaves its result uninitialised, so only parse when the variable is set
    const string tquit = ofGetEnv( "TRACKER_QUIT_AFTER" );
    mAutoQuitSec = tquit.empty() ? 0.f : ofToFloat( tquit );
    mDemo = ofGetEnv( "FEMTO_DEMO" ) != "";
#ifdef NDEBUG
    mDebug = false;   // release build: no diagnostics
#else
    mDebug = true;    // debug build: once a second, where the skeletons are lost
#endif
    if( mDemo ) {
        // two stand-in cameras, calibrated, watching someone walk about; the real ones in the settings are left be
        for( auto& c : mCams ) c->enabled = false;
        for( int i = 0; i < 2; i++ ) {
            mCams[i]->fake = true;
            mCams[i]->K.width = 1280; mCams[i]->K.height = 720; mCams[i]->K.fx = mCams[i]->K.fy = 900.f;
            mCams[i]->leveled = true;
        }
        mCams[1]->toRef = glm::translate( glm::mat4( 1.f ), glm::vec3( 2.4f, 0.f, 4.2f )) * glm::rotate( glm::mat4( 1.f ), glm::radians( 150.f ), glm::vec3( 0.f, 1.f, 0.f ));
        mCams[1]->calibrated = true;
    }

    mOutRunning = true;
    mOutThread = std::thread( &ofApp::outputLoop, this );

    // every Femto there is: those found are started (not in the demo, which has no cameras)
    if( !mDemo ) mFinder.start( 3.f );
}

//--------------------------------------------------------------
void ofApp::setupCamera( Camera& aCam ) {
    aCam.params.setName( "Camera " + ofToString( aCam.index ));
    aCam.params.add( aCam.enabled.set( "Enabled", aCam.index == 0 ));
    aCam.params.add( aCam.ip.set( "Address", "192.168.1." + ofToString( 10 + aCam.index )));
    aCam.params.add( aCam.levelFromImu.set( "Level from accelerometer", true ));
    aCam.params.add( aCam.invertImu.set( "Camera Upside Down", false ));
}

//--------------------------------------------------------------
void ofApp::loadSettings() {
    ofXml txml;
    if( txml.load( "settings.xml" )) {
        ofDeserialize( txml, mAllParams );
        ofLogNotice( "FemtoSkeletonOsc" ) << "settings loaded";
    }
}

//--------------------------------------------------------------
void ofApp::saveSettings() {
    ofXml txml;
    ofSerialize( txml, mAllParams );
    txml.save( "settings.xml" );
    mSettingsSavedAt = ofGetElapsedTimef();
}

//--------------------------------------------------------------
void ofApp::loadCalibration() {
    if( !ofFile::doesFileExist( ofToDataPath( "calibration.json" ))) return;
    ofJson tj = ofLoadJson( "calibration.json" );
    if( tj.is_null() || !tj.contains( "cameras" )) return;
    for( auto& te : tj["cameras"] ) {
        int ti = te.value( "index", -1 );
        if( ti < 1 || ti >= kMaxCameras || !te.contains( "toRef" ) || te["toRef"].size() != 16 ) continue;
        Camera& tc = *mCams[ti];
        for( int k = 0; k < 16; k++ ) tc.toRef[k / 4][k % 4] = te["toRef"][k].get< float >();
        tc.calibrated = true;
        tc.calRms = te.value( "rms", 0.f );
        tc.calInliers = te.value( "inliers", 0 );
        tc.calMessage = "loaded";
    }
    ofLogNotice( "FemtoSkeletonOsc" ) << "calibration loaded";
}

//--------------------------------------------------------------
void ofApp::saveCalibration() {
    ofJson tj;
    tj["cameras"] = ofJson::array();
    for( int i = 1; i < kMaxCameras; i++ ) {
        const Camera& tc = *mCams[i];
        if( !tc.calibrated ) continue;
        ofJson te;
        te["index"] = i;
        te["rms"] = tc.calRms;
        te["inliers"] = tc.calInliers;
        te["toRef"] = ofJson::array();
        for( int k = 0; k < 16; k++ ) te["toRef"].push_back( tc.toRef[k / 4][k % 4] );
        tj["cameras"].push_back( te );
    }
    ofSavePrettyJson( ofToDataPath( "calibration.json", true ), tj );
}

//--------------------------------------------------------------
void ofApp::setupTracker( Camera& aCam ) {
    ofLogNotice( "FemtoSkeletonOsc" ) << "camera " << aCam.index << ": pose model " << ( useYolo ? "YOLO11n-pose" : "RTMW" );
    if( useYolo ) {
        // YOLO11n-pose: people and their 17 keypoints in one pass. Fetched the first time with the system's curl.
        const string tpath = ofToDataPath( "models/YoloPose/yolo11n-pose.onnx", true );
        if( !ofFile::doesFileExist( tpath )) {
            ofDirectory::createDirectory( ofFilePath::getEnclosingDirectory( tpath ), false, true );
            ofLogNotice( "FemtoSkeletonOsc" ) << "downloading yolo11n-pose.onnx";
            const string tcmd = "curl -fsSL -m 600 -o \"" + tpath + "\" https://huggingface.co/MikeLud/ObjectDetectionYOLO11-ONNX/resolve/main/yolo11n-pose.onnx";
            if( system( tcmd.c_str()) != 0 ) ofFile::removeFile( tpath );
        }
        auto tyolo = std::make_shared< ofxYoloPose >();
        ofxYoloPose::Settings tys;
        tys.modelPath = tpath;
        if( mDebug ) tys.logLevel = OF_LOG_VERBOSE;   // debug builds: what it decodes, every couple of seconds
        if( tyolo->setup( tys )) {
            std::lock_guard< std::mutex > tlk( aCam.mtx );
            aCam.yolo = tyolo;
        } else {
            ofLogError( "FemtoSkeletonOsc" ) << "camera " << aCam.index << ": YOLO11n-pose did not load";
        }
        return;
    }
    // The detector finds the people, the whole-body pose model puts 133 joints on each (body, feet, face, hands).
    // The models are downloaded the first time.
    ofHttpRequest treq;
    treq.timeoutSeconds = 60 * 15;
    ofxRtmPoseTracker::Settings tsettings;
    ofLogNotice( "FemtoSkeletonOsc" ) << "camera " << aCam.index << ": loading models (downloaded on the first run)";
    // the tiny detector: a 416x416 input instead of yolox_m's 640x640, for speed
    tsettings.detectorModelPath = ofxRtmModelLoader::loadModel( "yolox_tiny", "models/RtmDetectors",
        "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/yolox_tiny_8xb8-300e_humanart-6f3252f9.zip", treq );
    // RTMW-x-l: the whole-body model trained on 14 datasets together (Cocktail14), better on hands, crossed arms
    // and people side on than RTMPose-l on COCO-WholeBody alone; the same 133 keypoints
    tsettings.poseModelPath = ofxRtmModelLoader::loadModel( "rtmw-x-l-384x288", "models/RtmPoseWholeBody",
        "https://download.openmmlab.com/mmpose/v1/projects/rtmw/onnx_sdk/rtmw-dw-x-l_simcc-cocktail14_270e-384x288_20231122.zip", treq );
    tsettings.logLevel = OF_LOG_NOTICE;
    auto ttracker = std::make_shared< ofxRtmPoseTracker >();
    if( ttracker->setup( tsettings )) {
        // run by the camera's thread (cameraLoop), not on a thread of its own or from the app's update
        ttracker->setEnableUpdateListeners( false );
        std::lock_guard< std::mutex > tlk( aCam.mtx );
        aCam.tracker = ttracker;
    } else {
        ofLogError( "FemtoSkeletonOsc" ) << "camera " << aCam.index << ": the pose tracker did not load";
    }
}

//--------------------------------------------------------------
void ofApp::applyOsc() {
    std::lock_guard< std::mutex > tlk( mOscMutex );
    mSkeleton.setup( oscHost, oscPort );
    ofLogNotice( "FemtoSkeletonOsc" ) << "sending OSC to " << oscHost.get() << ":" << oscPort.get();
}

//--------------------------------------------------------------
// Runs on a worker thread: a camera that is off or on another subnet only gives up after a few seconds.
void ofApp::connectCamera( Camera& aCam ) {
    if( mDemo ) return;   // the demo's cameras are stand-ins: no real camera is opened, whatever the settings say
    ofxOrbbecDevice::Options topts;
    topts.colorWidth = colorWidth;
    topts.colorHeight = colorHeight;
    // fps left at 0: the fastest the camera offers. Depth at 640x576 (narrow, unbinned, still 30 fps) rather than
    // the fastest 320x288: registered into 1280x720 colour, the binned depth leaves a hand or a finger too few
    // depth pixels of its own, and it takes the wall behind.
    topts.depthWidth = 640;
    topts.depthHeight = 576;
    topts.alignDepthToColor = true;   // a colour pixel indexes the depth image directly
    topts.netAddress = aCam.ip.get();
    topts.netPort = cameraPort;
    const string taddress = aCam.ip.get();
    Camera* tcam = &aCam;
    aCam.connecting = std::async( std::launch::async, [tcam, topts, taddress]() {
        if( !tcam->device.open( "", ofxOrbbecDevice::SyncMode::Standalone, topts )) {
            ofLogWarning( "FemtoSkeletonOsc" ) << "no Femto Mega answered at " << taddress << "; trying again";
            return false;
        }
        return true;
    });
}

//--------------------------------------------------------------
void ofApp::update() {
    if( mAutoQuitSec > 0 && ofGetElapsedTimef() > mAutoQuitSec ) {
        size_t tin = 0, tsent = 0;
        for( auto& c : mCams ) tin += c->framesIn;
        {
            std::lock_guard< std::mutex > tlk( mOscMutex );
            tsent = mSkeleton.getNumSent();
        }
        ofLogNotice( "FemtoSkeletonOsc" ) << "quitting: " << tin << " frames in, " << mBodiesSent << " bodies and "
                                          << tsent << " OSC messages sent; " << ofGetFrameRate() << " fps";
        ofExit();
        return;
    }

    // FEMTO_PLAY=<file in bin/data>: camera 0 plays that recording from the start.
    // FEMTO_RECORD=1: camera 0 is recorded from when it opens (with TRACKER_QUIT_AFTER, for a take of a set length).
    static const string tplayEnv = ofGetEnv( "FEMTO_PLAY" ), trecordEnv = ofGetEnv( "FEMTO_RECORD" );
    static bool tplayStarted = false, trecordStarted = false;
    if( !tplayEnv.empty() && !tplayStarted ) {
        tplayStarted = true;
        mCams[0]->selectedTake = tplayEnv;
        playTake( *mCams[0], tplayEnv );
    }
    if( !trecordEnv.empty() && !trecordStarted && mCams[0]->running && !mCams[0]->player ) {
        trecordStarted = true;
        const string tpath = "femto_takes/cam0_" + ofGetTimestampString( "%Y%m%d_%H%M%S" ) + ".femto";
        mCams[0]->recorder.start( ofToDataPath( tpath, true ), mCams[0]->device.getSerial(), mCams[0]->K );
    }

    adoptFound();

    for( auto& c : mCams ) {
        Camera& cam = *c;
        if( !cam.enabled ) {
            if( cam.player ) stopTake( cam );
            if( cam.device.isOpen() && !cam.connecting.valid() ) {
                stopCamera( cam );
                cam.device.close();
                std::lock_guard< std::mutex > tlk( cam.mtx );
                cam.bodies.clear();
                cam.poses2d.clear();
            }
            continue;
        }
        // a failed load (no network, say) is a download each time: try again every 30 s, not every frame
        if(( useYolo ? !cam.yolo : !cam.tracker ) && ofGetElapsedTimef() - cam.lastTrackerTry > 30.f ) {
            cam.lastTrackerTry = ofGetElapsedTimef();
            setupTracker( cam );
        }
        if( cam.player ) {
            // a recording in place of the camera: nothing to connect to
        } else if( cam.connecting.valid() ) {
            if( cam.connecting.wait_for( std::chrono::seconds( 0 )) != std::future_status::ready ) continue;
            if( cam.connecting.get() ) {
                cam.K = cam.device.getColorIntrinsics();
                ofLogNotice( "FemtoSkeletonOsc" ) << "camera " << cam.index << ": Femto " << cam.device.getSerial() << " at " << cam.ip.get()
                                                  << " open, colour " << cam.K.width << "x" << cam.K.height << " fx " << cam.K.fx << " fy " << cam.K.fy;
                startCamera( cam );
            }
            cam.lastTry = ofGetElapsedTimef();
        }
        if( !cam.player && !cam.device.isOpen() ) {
            if( ofGetElapsedTimef() - cam.lastTry > 3.f ) connectCamera( cam );
            continue;
        }
        // the newest picture, for the texture
        std::lock_guard< std::mutex > tlk( cam.mtx );
        if( cam.shownNew ) {
            if( cam.tex.isAllocated() && ( cam.tex.getWidth() != cam.shown.getWidth() || cam.tex.getHeight() != cam.shown.getHeight() )) cam.tex.clear();
            cam.tex.loadData( cam.shown );
            cam.shownNew = false;
        }
    }

    if( mDemo ) {
        updateDemo();
        wakeOutput();
    }
}

//--------------------------------------------------------------
// The cameras the search found, started: one already given a place (its address in a camera's settings) is started
// there unless it was stopped with its button; a new one takes the first camera that is not in use.
void ofApp::adoptFound() {
    if( ofGetElapsedTimef() - mAdoptedAt < 1.f ) return;
    mAdoptedAt = ofGetElapsedTimef();
    for( auto& f : mFinder.getFound() ) {
        if( f.ip.empty() ) continue;   // on USB: the cameras are opened by address here
        Camera* tplace = nullptr;
        for( auto& c : mCams ) if( c->ip.get() == f.ip ) { tplace = c.get(); break; }
        if( !tplace ) {
            for( auto& c : mCams ) {
                if( !c->enabled && !c->userStopped && !c->device.isOpen() && !c->player ) { tplace = c.get(); break; }
            }
            if( !tplace ) continue;   // every slot in use
            tplace->ip = f.ip;
        }
        if( !tplace->enabled && !tplace->userStopped ) {
            tplace->enabled = true;
            tplace->lastTry = -100.f;
            ofLogNotice( "FemtoSkeletonOsc" ) << "found " << f.name << " " << f.serial << " at " << f.ip << ": camera " << tplace->index;
        }
    }
}

void ofApp::startCameraByUser( Camera& aCam ) {
    aCam.userStopped = false;
    aCam.enabled = true;
    aCam.lastTry = -100.f;
}

// update() lets the camera go once it is not enabled; a recording playing in its place stops too
void ofApp::stopCameraByUser( Camera& aCam ) {
    aCam.userStopped = true;
    aCam.recorder.stop();
    if( aCam.player ) stopTake( aCam );
    aCam.enabled = false;
}

//--------------------------------------------------------------
// The recordings of the cameras (bin/data/femto_takes/*.femto), newest first.
void ofApp::listTakes() {
    mTakesListedAt = ofGetElapsedTimef();
    std::vector< std::pair< long long, string >> tfound;
    ofDirectory td( ofToDataPath( "femto_takes", true ));
    if( td.exists() ) {
        td.allowExt( "femto" );
        td.listDir();
        for( auto& f : td ) tfound.push_back( { (long long)std::filesystem::last_write_time( f.path() ).time_since_epoch().count(), "femto_takes/" + f.getFileName() } );
    }
    std::sort( tfound.begin(), tfound.end(), []( auto& a, auto& b ) { return a.first > b.first; } );
    mTakes.clear();
    for( auto& f : tfound ) mTakes.push_back( f.second );
}

// A recording played in place of the camera: the camera is let go, the worker takes its frames from the recording.
void ofApp::playTake( Camera& aCam, const string& aPath ) {
    stopCamera( aCam );
    aCam.recorder.stop();
    if( aCam.connecting.valid() ) aCam.connecting.wait();
    if( aCam.device.isOpen() ) aCam.device.close();
    auto tplayer = std::make_shared< ofxOrbbecPlayer >();
    if( !tplayer->open( ofToDataPath( aPath, true ))) return;
    aCam.K = tplayer->getColorIntrinsics();
    {
        std::lock_guard< std::mutex > tlk( aCam.mtx );
        aCam.player = tplayer;
        aCam.bodies.clear();
        aCam.boxes.clear();
        aCam.poses2d.clear();
    }
    startCamera( aCam );
}

// Back to the camera: the next update() connects to it again.
void ofApp::stopTake( Camera& aCam ) {
    stopCamera( aCam );
    {
        std::lock_guard< std::mutex > tlk( aCam.mtx );
        aCam.player.reset();
        aCam.bodies.clear();
        aCam.boxes.clear();
        aCam.poses2d.clear();
    }
    aCam.lastTry = -100.f;
}

//--------------------------------------------------------------
bool ofApp::recordingAny() const {
    for( auto& c : mCams ) if( c->recorder.isRecording() ) return true;
    return false;
}

void ofApp::toggleRecording() {
    if( recordingAny() ) {
        for( auto& c : mCams ) c->recorder.stop();
        mTakesListedAt = -100.f;
        return;
    }
    // one time for all the cameras' files, so a take's files go together
    const string tstamp = ofGetTimestampString( "%Y%m%d_%H%M%S" );
    for( auto& c : mCams ) {
        Camera& cam = *c;
        if( !cam.enabled || cam.player || !cam.device.isOpen() ) continue;
        const string tpath = "femto_takes/cam" + ofToString( cam.index ) + "_" + tstamp + ".femto";
        if( cam.recorder.start( ofToDataPath( tpath, true ), cam.device.getSerial(), cam.K )) cam.selectedTake = tpath;
    }
}

//--------------------------------------------------------------
void ofApp::startCamera( Camera& aCam ) {
    if( aCam.running ) return;
    aCam.running = true;
    aCam.worker = std::thread( &ofApp::cameraLoop, this, std::ref( aCam ));
}

void ofApp::stopCamera( Camera& aCam ) {
    aCam.running = false;
    if( aCam.worker.joinable() ) aCam.worker.join();
}

//--------------------------------------------------------------
// A camera's own thread: its newest frame, the poses in it, and that frame's skeletons from that frame's own depth
// (the tracker is run here, not on a thread of its own, so the poses and the depth are always the same moment).
// Frames that come while the models run are passed over: the next one taken is the newest.
void ofApp::cameraLoop( Camera& aCam ) {
    ofEventArgs targs;
    // a recording played in place of the camera, if one is (it is set before this thread starts)
    std::shared_ptr< ofxOrbbecPlayer > tplayer;
    {
        std::lock_guard< std::mutex > tlk( aCam.mtx );
        tplayer = aCam.player;
    }
    while( aCam.running ) {
        if( tplayer ) tplayer->update(); else aCam.device.update();
        const ofxOrbbecFrame& tframe = tplayer ? tplayer->getFrame() : aCam.device.getFrame();
        if( !tframe.isNew || !tframe.color.isAllocated() ) {
            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ));
            continue;
        }
        aCam.framesIn++;
        // the colour is copied (it goes on to the texture); the depth is read where it is, which holds until the
        // next device.update(), on this thread
        ofPixels tcolor = tframe.color;
        const ofShortPixels& tdepth = tframe.depth;

        // the camera's pitch and roll: from its accelerometer when it has one
        glm::vec3 tup;
        glm::mat3 tlevel( 1.f );
        bool tleveled = false, timu = false;
        float tpitch = 0.f, troll = 0.f;
        glm::vec3 traw;
        const bool thasRaw = tplayer ? tplayer->getUp( traw ) : aCam.device.getUp( traw );
        // recording: the frame as the camera gave it, with the accelerometer as it read
        if( !tplayer && aCam.recorder.isRecording() ) aCam.recorder.add( tcolor, tdepth, thasRaw, traw );
        if( aCam.levelFromImu && thasRaw ) {
            tup = traw;
            if( aCam.invertImu ) tup = -tup;
            timu = true;
            tpitch = glm::degrees( std::asin( glm::clamp( tup.z, -1.f, 1.f )));
            troll = glm::degrees( std::atan2( tup.x, -tup.y ));
            tleveled = levelMatrix( tup, tlevel );
        }

        std::shared_ptr< ofxRtmPoseTracker > ttracker;
        std::shared_ptr< ofxYoloPose > tyolo;
        {
            std::lock_guard< std::mutex > tlk( aCam.mtx );
            ttracker = aCam.tracker;
            tyolo = aCam.yolo;
        }
        if( useYolo ) ttracker.reset(); else tyolo.reset();
        bool tnew = false;
        vector< FemtoSkeleton::Body > tbodies;
        vector< FemtoSkeleton::Box > tboxes;
        vector< Camera::Pose2D > tposes;
        // the people in this frame, as the RTM tracker gives them (YOLO's are put in the same form): keypoints in
        // colour pixels, so build() and buildBox() take either
        vector< std::shared_ptr< ofxRtmTrackedPose >> tfound;
        if( tyolo ) {
            tnew = tyolo->run( tcolor );
            aCam.toTracker++;
            if( tnew ) {
                for( auto& yp : tyolo->getPoses() ) {
                    auto tp = std::make_shared< ofxRtmTrackedPose >();
                    tp->ID = yp.id;
                    tp->bFoundThisFrame = yp.foundThisFrame;
                    tp->keypoints.assign( yp.keypoints.begin(), yp.keypoints.end() );
                    tp->scores.assign( yp.scores.begin(), yp.scores.end() );
                    tp->box.outRect = yp.box;
                    tp->box.score = yp.score;
                    tfound.push_back( tp );
                }
            }
        } else if( ttracker ) {
            ttracker->update( targs );   // ages what it tracks, and clears the last frame's "new"
            // keypoints come back as pixels of the colour image
            ttracker->setOutRect( ofRectangle( 0, 0, tcolor.getWidth(), tcolor.getHeight() ));
            tnew = ttracker->run( tcolor ) && ttracker->hasNewData();
            aCam.toTracker++;
            if( tnew ) tfound = ttracker->getPoses();
        }
        {
            if( tnew ) {
                Camera::DebugCounts& d = aCam.dbg;
                for( auto& tpose : tfound ) {
                    if( !tpose ) continue;
                    if( mDebug ) d.poses++;
                    if( !tpose->bFoundThisFrame ) continue;
                    tposes.push_back( { tpose->ID, tpose->keypoints, tpose->scores, tpose->box.outRect, true } );
                    if( mDebug ) {
                        d.found++;
                        // the body keypoints build() uses: COCO 5..16
                        for( int k = 5; k <= 16 && k < (int)tpose->keypoints.size() && k < (int)tpose->scores.size(); k++ ) {
                            d.scoreMax = std::max( d.scoreMax, tpose->scores[k] );
                            if( tpose->scores[k] < mSkeleton.minScore ) continue;
                            d.scored++;
                            ofVec3f tp;
                            if( mSkeleton.unproject( ofVec2f( tpose->keypoints[k].x, tpose->keypoints[k].y ), tdepth, aCam.K, tp )) d.withDepth++;
                        }
                    }
                    FemtoSkeleton::Body tbody;
                    const bool tbuilt = mSkeleton.build( *tpose, tdepth, aCam.K, tbody, tleveled ? &tlevel : nullptr );
                    if( tbuilt ) {
                        tbodies.push_back( tbody );
                    } else {
                        // found, but too little of the skeleton placed (arms crossed, turned away, half out of view):
                        // the person still is somewhere, so their box goes out instead
                        FemtoSkeleton::Box tbox;
                        if( mSkeleton.buildBox( *tpose, tdepth, aCam.K, tbox, tleveled ? &tlevel : nullptr )) tboxes.push_back( tbox );
                        tposes.back().skeleton = false;
                    }
                    if( mDebug ) {
                        d.ids.insert( tpose->ID );
                        if( tbuilt ) { d.jointsMin = std::min( d.jointsMin, tbody.numSeen ); d.jointsMax = std::max( d.jointsMax, tbody.numSeen ); }
                        else d.notBuilt++;
                    }
                }
            }
        }

        if( mDebug && tnew ) {
            Camera::DebugCounts& d = aCam.dbg;
            d.updates++;
            d.built += (int)tbodies.size();
            if( ofGetElapsedTimef() - d.lastLog > 1.f ) {
                size_t tvalid = 0;
                const size_t tnpix = (size_t)tdepth.getWidth() * tdepth.getHeight();
                for( size_t i = 0; i < tnpix; i++ ) if( tdepth.getData()[i] ) tvalid++;
                ofLogNotice( "FemtoSkeletonOsc" ) << "debug cam " << aCam.index << ": " << ( aCam.framesIn - d.framesAtLog ) << " camera frames, "
                    << d.updates << " tracker updates, " << d.poses << " poses, "
                    << d.found << " found, body keypoints scored " << d.scored << " (max score " << d.scoreMax << "), with depth " << d.withDepth
                    << ", bodies built " << d.built << " | depth " << tdepth.getWidth() << "x" << tdepth.getHeight() << " "
                    << ( tnpix ? 100.0 * tvalid / tnpix : 0.0 ) << "% valid, colour " << tcolor.getWidth() << "x" << tcolor.getHeight()
                    << ", K " << aCam.K.width << "x" << aCam.K.height;
                string tids;
                for( auto i : d.ids ) tids += ofToString( i ) + " ";
                ofLogNotice( "FemtoSkeletonOsc" ) << "debug cam " << aCam.index << ": ids " << tids << "| joints per body "
                    << ( d.built ? ofToString( d.jointsMin ) + "-" + ofToString( d.jointsMax ) : string( "-" )) << ", found but not sent " << d.notBuilt;
                if( timu ) {
                    ofLogNotice( "FemtoSkeletonOsc" ) << "debug cam " << aCam.index << ": accelerometer up (colour camera frame, y down) "
                        << tup.x << " " << tup.y << " " << tup.z << ( aCam.invertImu ? ", inverted by the setting" : "" )
                        << ", pitch " << tpitch << " roll " << troll << ( tleveled ? " levelling" : "" );
                }
                const size_t tframes = aCam.framesIn;
                d = Camera::DebugCounts();
                d.framesAtLog = tframes;
                d.lastLog = ofGetElapsedTimef();
            }
        }

        // hand it all over
        {
            std::lock_guard< std::mutex > tlk( aCam.mtx );
            std::swap( aCam.shown, tcolor );
            aCam.shownNew = true;
            aCam.leveled = tleveled;
            if( tleveled ) aCam.levelM = tlevel;
            if( timu ) { aCam.pitchDeg = tpitch; aCam.rollDeg = troll; }
            if( tnew ) {
                aCam.poses2d = std::move( tposes );
                aCam.bodies = std::move( tbodies );
                aCam.boxes = std::move( tboxes );
                aCam.bodiesTime = ofGetElapsedTimef();
            }
        }
        if( tnew ) wakeOutput();
    }
}

//--------------------------------------------------------------
void ofApp::wakeOutput() {
    {
        std::lock_guard< std::mutex > tlk( mOutWakeMutex );
        mOutWake = true;
    }
    mOutCv.notify_one();
}

// The output thread: whenever a camera has new bodies, everything the calibrated cameras saw lately, in camera 0's
// frame, merged into people and sent.
void ofApp::outputLoop() {
    while( mOutRunning ) {
        {
            std::unique_lock< std::mutex > tlk( mOutWakeMutex );
            mOutCv.wait_for( tlk, std::chrono::milliseconds( 100 ), [&] { return mOutWake || !mOutRunning; } );
            if( !mOutWake ) continue;
            mOutWake = false;
        }
        const double tnow = ofGetElapsedTimef();

        // calibration points: a camera's person and camera 0's, at about the same moment
        vector< FemtoSkeleton::Body > tref;
        double trefTime = 0.0;
        {
            std::lock_guard< std::mutex > tlk( mCams[0]->mtx );
            tref = mCams[0]->bodies;
            trefTime = mCams[0]->bodiesTime;
        }
        for( int i = 1; i < kMaxCameras; i++ ) if( mCams[i]->collecting ) collectCalibration( *mCams[i], tref, trefTime );

        std::vector< SkeletonFusion::Detection > tdets;
        struct OutBox { uint64_t id; glm::vec3 centre, size; };
        std::vector< OutBox > tboxes;
        for( auto& c : mCams ) {
            Camera& cam = *c;
            std::lock_guard< std::mutex > tlk( cam.mtx );
            if( !cam.active() || ( cam.index > 0 && !cam.calibrated )) continue;
            if( tnow - cam.bodiesTime > 0.3 ) continue;
            // a box's id keeps clear of the merged people's: the camera's number, then the tracker's id
            for( auto& b : cam.boxes ) tboxes.push_back( { (uint64_t)cam.index * 1000000 + b.id, glm::vec3( cam.toRef * glm::vec4( b.centre, 1.f )), b.size } );
            for( auto& b : cam.bodies ) {
                SkeletonFusion::Detection td;
                td.camera = cam.index;
                td.body = b;
                for( int j = 0; j < FemtoSkeleton::TOTAL_JOINTS; j++ ) {
                    if( !b.seen[j] ) continue;
                    glm::vec3 tp = glm::vec3( cam.toRef * glm::vec4( vec3Of( b.cam[j] ), 1.f ));
                    td.body.cam[j] = ofVec3f( tp.x, tp.y, tp.z );
                }
                tdets.push_back( td );
            }
        }

        vector< FemtoSkeleton::Body > tout;
        {
            std::lock_guard< std::mutex > tlk( mFusionMutex );
            for( auto& t : mFusion.update( tdets, tnow )) {
                if( !t.updated ) continue;
                FemtoSkeleton::Body tb = t.body;
                if( mSkeleton.toSensorFrame( tb )) tout.push_back( tb );
            }
        }
        if( bSend && ( !tout.empty() || !tboxes.empty() )) {
            std::lock_guard< std::mutex > tlk( mOscMutex );
            for( auto& tb : tout ) {
                mSkeleton.send( tb );
                mBodiesSent++;
            }
            for( auto& tb : tboxes ) mSkeleton.sendBox( tb.id, tb.centre, tb.size );
        }
    }
}

//--------------------------------------------------------------
// FEMTO_DEMO: someone walks a loop and waves, seen by two stand-in cameras, each with a little joint noise, as
// if both were calibrated. For looking at the 3D view without cameras.
void ofApp::updateDemo() {
    using F = FemtoSkeleton;
    static std::mt19937 trng( 3 );
    std::normal_distribution< float > tnoise( 0.f, 0.012f );
    const float tt = ofGetElapsedTimef();
    const glm::vec3 thip( 0.9f * std::cos( tt * 0.5f ), floorBelow - 0.95f, 2.6f + 0.7f * std::sin( tt * 0.5f ));
    const float twave = std::sin( tt * 4.f );
    // joints, relative to the hip: x across, `up` metres above it
    struct J { int joint; float x, up, z; };
    const J tj[] = {
        { F::SPINE_BASE, 0, 0, 0 }, { F::SPINE_MID, 0, 0.25f, 0 }, { F::SPINE_SHOULDER, 0, 0.5f, 0 }, { F::NECK, 0, 0.6f, 0 }, { F::HEAD, 0, 0.78f, 0 },
        { F::SHOULDER_LEFT, -0.18f, 0.5f, 0 }, { F::ELBOW_LEFT, -0.24f, 0.25f, 0 }, { F::WRIST_LEFT, -0.22f, 0.03f, 0.05f }, { F::HAND_LEFT, -0.22f, -0.03f, 0.05f },
        { F::SHOULDER_RIGHT, 0.18f, 0.5f, 0 }, { F::ELBOW_RIGHT, 0.3f, 0.6f, 0 }, { F::WRIST_RIGHT, 0.32f + 0.08f * twave, 0.85f, 0 }, { F::HAND_RIGHT, 0.33f + 0.1f * twave, 0.93f, 0 },
        { F::HIP_LEFT, -0.1f, 0, 0 }, { F::KNEE_LEFT, -0.1f, -0.45f, 0 }, { F::ANKLE_LEFT, -0.1f, -0.88f, 0 }, { F::FOOT_LEFT, -0.1f, -0.93f, 0.1f },
        { F::HIP_RIGHT, 0.1f, 0, 0 }, { F::KNEE_RIGHT, 0.1f, -0.45f, 0 }, { F::ANKLE_RIGHT, 0.1f, -0.88f, 0 }, { F::FOOT_RIGHT, 0.1f, -0.93f, 0.1f },
    };
    for( int i = 0; i < 2; i++ ) {
        Camera& cam = *mCams[i];
        const glm::mat4 tinv = glm::inverse( cam.toRef );
        FemtoSkeleton::Body tb;
        tb.id = 5;
        for( auto& j : tj ) {
            glm::vec3 tp = thip + glm::vec3( j.x, -j.up, j.z );
            tp = glm::vec3( tinv * glm::vec4( tp, 1.f )) + glm::vec3( tnoise( trng ), tnoise( trng ), tnoise( trng ));
            tb.cam[j.joint] = ofVec3f( tp.x, tp.y, tp.z );
            tb.seen[j.joint] = true;
        }
        std::lock_guard< std::mutex > tlk( cam.mtx );
        cam.bodies.assign( 1, tb );
        cam.bodiesTime = ofGetElapsedTimef();
    }
}

//--------------------------------------------------------------
// While collecting: one person alone in view of both this camera and camera 0, at about the same moment, gives
// a point for every joint both place.
// On the output thread, with camera 0's bodies as it took them.
void ofApp::collectCalibration( Camera& aCam, const vector< FemtoSkeleton::Body >& aRefBodies, double aRefTime ) {
    if( !mCams[0]->device.isOpen() || aRefBodies.size() != 1 ) return;
    std::lock_guard< std::mutex > tlk( aCam.mtx );
    if( aCam.bodies.size() != 1 || aCam.bodiesTime == aCam.calTakenAt ) return;
    if( std::abs( aRefTime - aCam.bodiesTime ) > 0.1 ) return;
    aCam.calTakenAt = aCam.bodiesTime;
    const auto& ta = aCam.bodies[0];
    const auto& tb = aRefBodies[0];
    for( int j = 0; j < FemtoSkeleton::TOTAL_JOINTS; j++ )
        if( ta.seen[j] && tb.seen[j] ) aCam.cal.add( vec3Of( ta.cam[j] ), vec3Of( tb.cam[j] ));
}

//--------------------------------------------------------------
void ofApp::draw() {
    std::vector< Camera* > tshown;
    for( auto& c : mCams ) if( c->enabled && c->tex.isAllocated() ) tshown.push_back( c.get() );

    // the right of the window: each camera with its poses over it, in a row along the top, and the joined
    // space in 3D below, in perspective (orbits with the mouse) and from above
    ofRectangle tfull( 450, 10, ofGetWidth() - 460, ofGetHeight() - 20 );
    const bool tcams = !tshown.empty();
    // the 3D views on top, the camera pictures below them: up to four in a row, then more rows
    const int tcols = std::max( 1, std::min( 4, (int)tshown.size() ));
    const int trows = ( (int)tshown.size() + tcols - 1 ) / tcols;
    const float tcamH = tcams ? tfull.height * ( trows > 1 ? 0.45f : 0.3f ) : 0.f;
    ofRectangle tarea( tfull.x, tfull.getBottom() - tcamH, tfull.width, tcamH );
    const float tviewsH = tfull.height - tcamH - ( tcams ? 6.f : 0.f );
    const float thalf = ( tfull.width - 6.f ) * 0.5f;
    mViewRect = ofRectangle( tfull.x, tfull.y, thalf, tviewsH );
    const ofRectangle ttop( mViewRect.getRight() + 6.f, tfull.y, thalf, tviewsH );
    const float tw = tarea.width / tcols;
    const float th = trows > 0 ? tarea.height / trows : tarea.height;
    for( size_t i = 0; i < tshown.size(); i++ ) {
        Camera& cam = *tshown[i];
        ofRectangle ttile( tarea.x + ( i % tcols ) * tw, tarea.y + ( i / tcols ) * th, tw - 6, th - ( trows > 1 ? 6 : 0 ));
        ofRectangle timg( 0, 0, cam.tex.getWidth(), cam.tex.getHeight() );
        timg.scaleTo( ttile, OF_SCALEMODE_FIT );
        timg.setPosition( ttile.x + ( ttile.width - timg.width ) * 0.5f, ttile.y );
        ofSetColor( 255 );
        cam.tex.draw( timg );
        // the poses the camera's thread found in it (not tracker->draw(): that thread is changing the tracker)
        vector< Camera::Pose2D > tposes;
        {
            std::lock_guard< std::mutex > tlk( cam.mtx );
            tposes = cam.poses2d;
        }
        const float ts = timg.width / std::max( 1.f, cam.tex.getWidth() );
        auto tat = [&]( const glm::vec2& k ) { return glm::vec2( timg.x + k.x * ts, timg.y + k.y * ts ); };
        // COCO body: face, arms, shoulders and hips, legs
        static const int tcoco[][2] = { { 0, 1 }, { 0, 2 }, { 1, 3 }, { 2, 4 }, { 5, 7 }, { 7, 9 }, { 6, 8 }, { 8, 10 }, { 5, 6 }, { 5, 11 },
                                        { 6, 12 }, { 11, 12 }, { 11, 13 }, { 13, 15 }, { 12, 14 }, { 14, 16 } };
        for( auto& tp : tposes ) {
            const ofColor tc = ofColor::fromHsb( ( tp.id * 53 ) % 255, 160, 255 );
            if( !tp.skeleton ) {
                // found, but no skeleton: sent as a box
                ofNoFill();
                ofSetLineWidth( 2.f );
                ofSetColor( optheme::boxPerson );
                ofDrawRectangle( tat( glm::vec2( tp.box.x, tp.box.y )), tp.box.width * ts, tp.box.height * ts );
                ofSetLineWidth( 1.f );
                ofFill();
                ofDrawBitmapString( "box", tat( glm::vec2( tp.box.x, tp.box.y )) + glm::vec2( 4, 14 ));
            }
            auto good = [&]( int k ) { return k < (int)tp.keypoints.size() && k < (int)tp.scores.size() && tp.scores[k] >= mSkeleton.minScore; };
            ofSetLineWidth( 2.f );
            ofSetColor( tc );
            for( auto& tb : tcoco ) if( good( tb[0] ) && good( tb[1] )) ofDrawLine( tat( tp.keypoints[tb[0]] ), tat( tp.keypoints[tb[1]] ));
            ofSetLineWidth( 1.f );
            for( int k = 0; k < (int)tp.keypoints.size(); k++ ) if( good( k )) ofDrawCircle( tat( tp.keypoints[k] ), k < 17 ? 3.f : 1.5f );
        }
        ofSetColor( optheme::camera( cam.index ) );
        ofDrawBitmapString( "camera " + ofToString( cam.index ) + ( cam.index > 0 && !cam.calibrated ? "  (not calibrated)" : "" ), timg.x + 6, timg.y + 14 );
    }

    draw3d( mViewRect, false );
    draw3d( ttop, true );

    // the controls, down the left: the settings above, the cameras below
    gui.begin();
    const float tgap = 10.f, tpanelW = 430.f, tpanelH = ( ofGetHeight() - 3.f * tgap ) * 0.5f;
    drawSettingsWindow( ofRectangle( tgap, tgap, tpanelW, tpanelH ));
    drawCamerasWindow( ofRectangle( tgap, 2.f * tgap + tpanelH, tpanelW, tpanelH ));
    gui.end();

    // FEMTO_SHOT=<file.png>: save what is on screen after a few seconds, then quit
    static const string tshot = ofGetEnv( "FEMTO_SHOT" );
    if( !tshot.empty() && ofGetElapsedTimef() > 6.f ) {
        ofImage timg;
        timg.grabScreen( 0, 0, ofGetWidth(), ofGetHeight() );
        timg.save( tshot );
        ofExit( 0 );
    }
}

//--------------------------------------------------------------
// Output, recording, and how people are found, merged and sent.
void ofApp::drawSettingsWindow( const ofRectangle& aArea ) {
    guiSettings.windowPos = aArea.getPosition();
    guiSettings.windowSize = glm::vec2( aArea.width, aArea.height );
    ofxImGui::BeginWindow( "Settings", guiSettings, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

    int tfused = 0;
    size_t tsent = 0;
    {
        std::lock_guard< std::mutex > tlk( mFusionMutex );
        for( auto& t : mFusion.tracks() ) if( ofGetElapsedTimef() - t.lastSeen < 0.5 ) tfused++;
    }
    {
        std::lock_guard< std::mutex > tlk( mOscMutex );
        tsent = mSkeleton.getNumSent();
    }
    ImGui::Text( "people now %d   sent %zu   OSC messages %zu   app %.0f fps", tfused, mBodiesSent.load(), tsent, ofGetFrameRate() );
    ImGui::Separator();
    if( ImGui::CollapsingHeader( params.getName().c_str(), ImGuiTreeNodeFlags_DefaultOpen )) addParams( params );
    if( ImGui::Button( "Apply OSC address" )) applyOsc();
    ImGui::SameLine();
    if( ImGui::Button( "Floor from feet" )) floorFromFeet();
    ImGui::SameLine();
    if( ImGui::Button( "Save settings" )) saveSettings();
    // a moment's confirmation, so a press is not left wondering
    if( ofGetElapsedTimef() - mSettingsSavedAt < 3.f ) { ImGui::SameLine(); ImGui::TextColored( optheme::statusOk(), "saved" ); }
    // record every connected camera (a file each), or stop
    {
        const bool trec = recordingAny();
        float tsec = 0.f;
        for( auto& tc : mCams ) if( tc->recorder.isRecording() ) tsec = std::max( tsec, (float)tc->recorder.getSeconds() );
        if( trec ) {
            if( optheme::actionButton(( "Stop recording  " + ofToString( tsec, 1 ) + " s  (r)" ).c_str(), optheme::dangerFill )) toggleRecording();
        } else if( ImGui::Button( "Record  (r)", ImVec2( -1, 0 ))) {
            toggleRecording();
        }
    }
    if( ImGui::CollapsingHeader( mFusion.params.getName().c_str() )) addParams( mFusion.params );
    if( ImGui::CollapsingHeader( mSkeleton.getParams().getName().c_str() )) addParams( mSkeleton.getParams() );
    if( useYolo && mCams[0]->yolo ) {
        if( ImGui::CollapsingHeader( mCams[0]->yolo->params.getName().c_str() )) addParams( mCams[0]->yolo->params );
    } else if( !useYolo && mCams[0]->tracker ) {
        auto& tparams = mCams[0]->tracker->getParams( "Pose tracker" );
        if( ImGui::CollapsingHeader( tparams.getName().c_str() )) addParams( tparams );
    }
    ofxImGui::EndWindow( guiSettings );
}

//--------------------------------------------------------------
// The cameras: what the search found, a section for each camera that is open (live, or playing a recording), a line
// for each starting or stopped, and a camera to add by its address.
void ofApp::drawCamerasWindow( const ofRectangle& aArea ) {
    guiSettings.windowPos = aArea.getPosition();
    guiSettings.windowSize = glm::vec2( aArea.width, aArea.height );
    ofxImGui::BeginWindow( "Cameras", guiSettings, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoMove | ImGuiWindowFlags_NoResize );

    const auto tfound = mFinder.getFound();
    string tlist;
    for( auto& f : tfound ) tlist += ( tlist.empty() ? "" : ", " ) + f.serial + ( f.ip.empty() ? " (USB)" : " " + f.ip );
    ImGui::PushStyleColor( ImGuiCol_Text, optheme::im( optheme::textMuted ));
    if( mDemo ) ImGui::TextWrapped( "demo: two stand-in cameras, calibrated, watching someone walk a loop and wave" );
    else if( tfound.empty() ) ImGui::TextWrapped( mFinder.sinceSearch() > 1e8f ? "looking for cameras..." : "no cameras found on the network yet" );
    else ImGui::TextWrapped( "found: %s", tlist.c_str() );
    ImGui::PopStyleColor();

    // open: their sections
    for( auto& c : mCams ) if( c->device.isOpen() || c->player ) drawCameraPanel( *c );

    // starting, or stopped: a line each
    for( auto& c : mCams ) {
        Camera& cam = *c;
        if( cam.device.isOpen() || cam.player ) continue;
        if( !cam.enabled && !cam.userStopped ) continue;
        ImGui::PushID( cam.index );
        ImGui::AlignTextToFramePadding();
        if( cam.enabled ) {
            ImGui::TextColored( optheme::statusWarning(), "Camera %d  %s  %s", cam.index, cam.ip.get().c_str(), cam.connecting.valid() ? "connecting..." : "not answering yet" );
            ImGui::SameLine( aArea.width - 80.f );
            if( ImGui::Button( "Stop", ImVec2( 60.f, 0 ))) stopCameraByUser( cam );
        } else {
            ImGui::TextColored( optheme::im( optheme::textMuted ), "Camera %d  %s  stopped", cam.index, cam.ip.get().c_str() );
            ImGui::SameLine( aArea.width - 80.f );
            if( ImGui::Button( "Start", ImVec2( 60.f, 0 ))) startCameraByUser( cam );
        }
        ImGui::PopID();
    }

    // one the search cannot see (another subnet, say), by its address
    ImGui::Separator();
    ImGui::AlignTextToFramePadding();
    ImGui::TextUnformatted( "add a camera" );
    ImGui::SameLine();
    ImGui::SetNextItemWidth( 150.f );
    ImGui::InputText( "##addIp", mAddIp, sizeof( mAddIp ));
    ImGui::SameLine();
    if( ImGui::Button( "Add" ) && mAddIp[0] ) {
        Camera* tplace = nullptr;
        for( auto& c : mCams ) if( c->ip.get() == mAddIp ) { tplace = c.get(); break; }
        if( !tplace ) for( auto& c : mCams ) if( !c->enabled && !c->device.isOpen() && !c->player ) { tplace = c.get(); break; }
        if( tplace ) {
            tplace->ip = string( mAddIp );
            startCameraByUser( *tplace );
            mAddIp[0] = 0;
        }
    }
    ofxImGui::EndWindow( guiSettings );
}

//--------------------------------------------------------------
void ofApp::drawCameraPanel( Camera& aCam ) {
    ImGui::PushID( aCam.index );
    const string tname = "Camera " + ofToString( aCam.index ) + ( aCam.index == 0 ? " (reference)" : "" ) + "  " + ( aCam.player ? string( "recording" ) : aCam.device.getSerial() );
    if( ImGui::CollapsingHeader(( tname + "###cam" ).c_str(), ImGuiTreeNodeFlags_DefaultOpen )) {
        if( optheme::actionButton( "Stop this camera", optheme::dangerFill )) stopCameraByUser( aCam );
        ImGui::TextColored( optheme::im( optheme::textMuted ), "address %s", aCam.ip.get().c_str() );
        optheme::toggle( aCam.levelFromImu );
        optheme::toggle( aCam.invertImu );
        float tpitch, troll;
        bool tleveled;
        size_t tnbodies;
        {
            std::lock_guard< std::mutex > tlk( aCam.mtx );
            tpitch = aCam.pitchDeg; troll = aCam.rollDeg; tleveled = aCam.leveled; tnbodies = aCam.bodies.size();
        }
        if( aCam.device.isOpen() ) {
            ImGui::Text( "%s  %dx%d  frames %zu  to tracker %zu  %.0f fps", aCam.device.getSerial().c_str(), aCam.K.width, aCam.K.height,
                         aCam.framesIn.load(), aCam.toTracker.load(), useYolo ? ( aCam.yolo ? aCam.yolo->getFps() : 0.f ) : ( aCam.tracker ? aCam.tracker->getFps() : 0.f ));
            if( aCam.device.hasImu() ) {
                ImGui::Text( "accelerometer: pitch %.1f  roll %.1f degrees%s", tpitch, troll, tleveled ? "  (levelling)" : "" );
            } else {
                ImGui::TextColored( optheme::statusWarning(), "no accelerometer reading: not levelled" );
            }
            ImGui::Text( "bodies now %zu", tnbodies );
        } else if( aCam.player ) {
            ImGui::Text( "recording  %dx%d  frames %zu  to tracker %zu", aCam.K.width, aCam.K.height, aCam.framesIn.load(), aCam.toTracker.load() );
            ImGui::Text( "bodies now %zu", tnbodies );
        } else if( aCam.enabled ) {
            if( aCam.connecting.valid() ) ImGui::TextColored( optheme::statusWarning(), "connecting to %s ...", aCam.ip.get().c_str() );
            else ImGui::TextColored( optheme::statusWarning(), "no camera at %s", aCam.ip.get().c_str() );
        }

        // Record what the camera gives (colour, depth, accelerometer), or play a recording in its place: the
        // skeletons are found in it as in a live camera.
        if( aCam.enabled && ImGui::TreeNode( "Record / play back" )) {
            if( aCam.player ) {
                auto& tp = *aCam.player;
                ImGui::TextColored( optheme::statusInfo(), "playing %s", ofFilePath::getFileName( aCam.selectedTake ).c_str() );
                const float tlen = (float)tp.getLength();
                ImGui::ProgressBar( tlen > 0.f ? (float)tp.getPosition() / tlen : 0.f, ImVec2( -1, 0 ),
                                    ( ofToString( tp.getPosition(), 1 ) + " / " + ofToString( tlen, 1 ) + " s" ).c_str() );
                bool tloop = tp.loop;
                if( optheme::toggle( "Loop", tloop, 110.f )) tp.loop = tloop;
                ImGui::SameLine();
                float tspeed = tp.speed;
                if( ImGui::SliderFloat( "Speed", &tspeed, 0.1f, 2.f, "%.2fx" )) tp.speed = tspeed;
                if( ImGui::Button( "Back to the camera" )) stopTake( aCam );
            } else if( aCam.device.isOpen() ) {
                if( aCam.recorder.isRecording() ) {
                    ImGui::TextColored( optheme::statusDanger(), "REC  %.1f s  %zu frames  %.0f MB%s", aCam.recorder.getSeconds(), aCam.recorder.getFramesWritten(),
                                        aCam.recorder.getBytesWritten() / 1048576.0, aCam.recorder.getFramesDropped() ? ( "  (" + ofToString( aCam.recorder.getFramesDropped() ) + " passed over)" ).c_str() : "" );
                    if( optheme::actionButton( "Stop recording", optheme::dangerFill )) { aCam.recorder.stop(); mTakesListedAt = -100.f; }
                } else if( ImGui::Button( "Record", ImVec2( -1, 0 ))) {
                    const string tpath = "femto_takes/cam" + ofToString( aCam.index ) + "_" + ofGetTimestampString( "%Y%m%d_%H%M%S" ) + ".femto";
                    if( aCam.recorder.start( ofToDataPath( tpath, true ), aCam.device.getSerial(), aCam.K )) aCam.selectedTake = tpath;
                }
            }
            if( ofGetElapsedTimef() - mTakesListedAt > 2.f ) listTakes();
            if( ImGui::ListBoxHeader( "##femtoTakes", ImVec2( -1, 110 ))) {
                for( auto& t : mTakes ) if( ImGui::Selectable( ofFilePath::getFileName( t ).c_str(), t == aCam.selectedTake )) aCam.selectedTake = t;
                ImGui::ListBoxFooter();
            }
            if( !aCam.selectedTake.empty() && !aCam.recorder.isRecording() && ImGui::Button( "Play it as this camera" )) playTake( aCam, aCam.selectedTake );
            ImGui::TreePop();
        }

        if( aCam.index > 0 ) {
            ImGui::Separator();
            // levelled by the accelerometer, both cameras differ only by a turn about the vertical and a shift
            bool tref;
            {
                std::lock_guard< std::mutex > tlk( mCams[0]->mtx );
                tref = mCams[0]->leveled;
            }
            const bool tyaw = tleveled && tref;
            // the points and the calibration are shared with the output thread
            std::lock_guard< std::mutex > tlk( aCam.mtx );
            if( aCam.calibrated ) {
                ImGui::Text( "calibrated: rms %.1f cm over %d points", aCam.calRms * 100.f, aCam.calInliers );
            } else {
                ImGui::TextColored( optheme::statusWarning(), "not calibrated: its people are not sent" );
            }
            optheme::toggle( "Collect points", aCam.collecting, 190.f );
            ImGui::SameLine();
            ImGui::Text( "%zu points, spread %.2f m", aCam.cal.size(), aCam.cal.spread( tyaw ));
            if( aCam.collecting ) ImGui::TextWrapped( "Walk about where this camera and camera 0 can both see you, alone in view." );
            if( ImGui::Button( "Solve" )) {
                Calibration::Result tr = aCam.cal.solve( tyaw );
                aCam.calMessage = tr.ok ? "solved" : tr.message;
                if( tr.ok ) {
                    aCam.toRef = tr.toRef;
                    aCam.calibrated = true;
                    aCam.calRms = tr.rms;
                    aCam.calInliers = tr.inliers;
                    aCam.collecting = false;
                    saveCalibration();
                }
            }
            ImGui::SameLine();
            if( ImGui::Button( "Clear points" )) aCam.cal.clear();
            ImGui::SameLine();
            // a calibration takes a walk to make again: the first press asks, a second within 3 s forgets
            const bool tasking = ofGetElapsedTimef() - aCam.forgetAskedAt < 3.f;
            if( tasking ) {
                ImGui::PushStyleColor( ImGuiCol_Button, optheme::im( optheme::dangerFill ));
                ImGui::PushStyleColor( ImGuiCol_ButtonHovered, optheme::im( optheme::dangerFill.getLerped( optheme::text, 0.12f )));
                ImGui::PushStyleColor( ImGuiCol_Text, optheme::im( optheme::textOn( optheme::dangerFill )));
            }
            const bool tforget = ImGui::Button( tasking ? "Really forget?###forget" : "Forget calibration###forget" );
            if( tasking ) ImGui::PopStyleColor( 3 );
            if( tforget ) {
                if( tasking ) {
                    aCam.calibrated = false;
                    aCam.toRef = glm::mat4( 1.f );
                    aCam.forgetAskedAt = -100.f;
                    saveCalibration();
                } else {
                    aCam.forgetAskedAt = ofGetElapsedTimef();
                }
            }
            if( !aCam.calMessage.empty() ) ImGui::TextWrapped( "%s", aCam.calMessage.c_str() );
            if( !tyaw ) ImGui::TextWrapped( "(a camera is not levelled by its accelerometer: solving for a full 3D pose, which needs more movement)" );
        }
    }
    ImGui::PopID();
}


//--------------------------------------------------------------
// camera 0's levelled frame (x right, y down, z forward) to the display's (x right, y away, z up)
static glm::vec3 displayOf( const glm::vec3& p ) { return glm::vec3( p.x, p.z, -p.y ); }

void ofApp::floorFromFeet() {
    float tmax = -1e9f;
    std::lock_guard< std::mutex > tlk( mFusionMutex );
    for( auto& t : mFusion.tracks() ) {
        if( ofGetElapsedTimef() - t.lastSeen > 0.5 ) continue;
        for( int j : { (int)FemtoSkeleton::FOOT_LEFT, (int)FemtoSkeleton::FOOT_RIGHT, (int)FemtoSkeleton::ANKLE_LEFT, (int)FemtoSkeleton::ANKLE_RIGHT } )
            if( t.body.seen[j] ) tmax = std::max( tmax, t.body.cam[j].y );
    }
    if( tmax > 0.f ) floorBelow = tmax;
}

//--------------------------------------------------------------
void ofApp::draw3d( const ofRectangle& aView, bool aTopDown ) {
    if( aView.width < 20 || aView.height < 20 ) return;
    ofSetColor( optheme::bg );
    ofDrawRectangle( aView );

    if( aTopDown ) {
        // straight down from far off with a narrow field of view, so it is nearly flat; away from camera 0 up the
        // view. It takes in the floor in front of camera 0 (7 m by 7.5 m) and every calibrated camera with 3 m of
        // its view, with a margin, so as cameras are placed it zooms out to keep them all in; the framing eases.
        glm::vec2 tlo( -3.5f, -0.5f ), thi( 3.5f, 7.f );
        for( auto& c : mCams ) {
            Camera& cam = *c;
            if( cam.index == 0 || !cam.active() || !cam.calibrated ) continue;
            glm::mat4 ttoRef;
            {
                std::lock_guard< std::mutex > tlk( cam.mtx );
                ttoRef = cam.toRef;
            }
            for( const glm::vec3& tp : { glm::vec3( 0.f ), glm::vec3( -1.5f, 0.f, 3.f ), glm::vec3( 1.5f, 0.f, 3.f ) } ) {
                const glm::vec3 td = displayOf( glm::vec3( ttoRef * glm::vec4( tp, 1.f )));
                tlo = glm::min( tlo, glm::vec2( td ));
                thi = glm::max( thi, glm::vec2( td ));
            }
        }
        tlo -= glm::vec2( 0.75f );
        thi += glm::vec2( 0.75f );
        const glm::vec2 tcentreWant = ( tlo + thi ) * 0.5f;
        const float thalfWant = std::max(( thi.y - tlo.y ) * 0.5f, ( thi.x - tlo.x ) * 0.5f * aView.height / aView.width );
        const float tk = 1.f - std::exp( -ofGetLastFrameTime() / 0.15f );
        if( mTopHalf <= 0.f ) { mTopCentre = tcentreWant; mTopHalf = thalfWant; }
        mTopCentre += ( tcentreWant - mTopCentre ) * tk;
        mTopHalf += ( thalfWant - mTopHalf ) * tk;
        const float tfov = 4.f;
        const float thalf = mTopHalf;
        const float tdist = thalf / std::tan( glm::radians( tfov * 0.5f ));
        const glm::vec3 ttarget( mTopCentre.x, mTopCentre.y, 0.f );
        mView.setFov( tfov );
        mView.setNearClip( tdist - 20.f );
        mView.setFarClip( tdist + 20.f );
        mView.setPosition( ttarget + glm::vec3( 0.f, 0.f, tdist ));
        mView.lookAt( ttarget, glm::vec3( 0.f, 1.f, 0.f ));
    } else {
        const float taz = glm::radians( mViewAz ), tel = glm::radians( mViewEl );
        mView.setNearClip( 0.05f );
        mView.setFarClip( 100.f );
        mView.setFov( 45.f );
        mView.setPosition( mViewTarget + mViewDist * glm::vec3( std::cos( tel ) * std::cos( taz ), std::cos( tel ) * std::sin( taz ), std::sin( tel )));
        mView.lookAt( mViewTarget, glm::vec3( 0.f, 0.f, 1.f ));
    }

    // the bones of a body, as pairs of joints
    using F = FemtoSkeleton;
    static const int tbones[][2] = {
        { F::SPINE_BASE, F::SPINE_MID }, { F::SPINE_MID, F::SPINE_SHOULDER }, { F::SPINE_SHOULDER, F::NECK }, { F::NECK, F::HEAD },
        { F::SPINE_SHOULDER, F::SHOULDER_LEFT }, { F::SHOULDER_LEFT, F::ELBOW_LEFT }, { F::ELBOW_LEFT, F::WRIST_LEFT }, { F::WRIST_LEFT, F::HAND_LEFT },
        { F::SPINE_SHOULDER, F::SHOULDER_RIGHT }, { F::SHOULDER_RIGHT, F::ELBOW_RIGHT }, { F::ELBOW_RIGHT, F::WRIST_RIGHT }, { F::WRIST_RIGHT, F::HAND_RIGHT },
        { F::SPINE_BASE, F::HIP_LEFT }, { F::HIP_LEFT, F::KNEE_LEFT }, { F::KNEE_LEFT, F::ANKLE_LEFT }, { F::ANKLE_LEFT, F::FOOT_LEFT },
        { F::SPINE_BASE, F::HIP_RIGHT }, { F::HIP_RIGHT, F::KNEE_RIGHT }, { F::KNEE_RIGHT, F::ANKLE_RIGHT }, { F::ANKLE_RIGHT, F::FOOT_RIGHT },
    };
    auto place = [&]( const FemtoSkeleton::Body& b, const glm::mat4& aToRef, int j ) {
        return displayOf( glm::vec3( aToRef * glm::vec4( vec3Of( b.cam[j] ), 1.f )));
    };
    auto drawBody = [&]( const FemtoSkeleton::Body& b, const glm::mat4& aToRef, float aJointR ) {
        for( auto& tb : tbones )
            if( b.seen[tb[0]] && b.seen[tb[1]] ) ofDrawLine( place( b, aToRef, tb[0] ), place( b, aToRef, tb[1] ));
        for( int j = 0; j < F::TOTAL_JOINTS; j++ )
            if( b.seen[j] ) ofDrawSphere( place( b, aToRef, j ), aJointR );
    };

    mView.begin( aView );
    ofEnableDepthTest();
    ofDisableLighting();

    // the floor: a grid of 1 m squares, below camera 0
    const float tfz = -floorBelow;
    ofSetColor( optheme::grid );
    for( int i = -6; i <= 6; i++ ) {
        ofDrawLine( (float)i, -1.f, tfz, (float)i, 11.f, tfz );
        ofDrawLine( -6.f, i + 5.f, tfz, 6.f, i + 5.f, tfz );
    }
    // camera 0's axes: x red, forward green, up blue
    ofSetLineWidth( 2.f );
    ofSetColor( optheme::axisX ); ofDrawLine( 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f );
    ofSetColor( optheme::axisY ); ofDrawLine( 0.f, 0.f, 0.f, 0.f, 0.5f, 0.f );
    ofSetColor( optheme::axisZ ); ofDrawLine( 0.f, 0.f, 0.f, 0.f, 0.f, 0.5f );
    ofSetLineWidth( 1.f );

    // the cameras: a frustum each, where it is in the joined space, in its own colour
    for( auto& c : mCams ) {
        Camera& cam = *c;
        std::lock_guard< std::mutex > tlk( cam.mtx );
        if( !cam.active() || ( cam.index > 0 && !cam.calibrated ) || cam.K.fx <= 0.f ) continue;
        const glm::mat3 tm = cam.leveled ? cam.levelM : glm::mat3( 1.f );
        auto corner = [&]( float sx, float sy ) {
            const float td = 0.6f;
            glm::vec3 p( sx * ( cam.K.width * 0.5f ) / cam.K.fx * td, sy * ( cam.K.height * 0.5f ) / cam.K.fy * td, td );
            return displayOf( glm::vec3( cam.toRef * glm::vec4( tm * p, 1.f )));
        };
        const glm::vec3 to = displayOf( glm::vec3( cam.toRef * glm::vec4( 0.f, 0.f, 0.f, 1.f )));
        const glm::vec3 tc[4] = { corner( -1.f, -1.f ), corner( 1.f, -1.f ), corner( 1.f, 1.f ), corner( -1.f, 1.f ) };
        ofSetColor( optheme::camera( cam.index ) );
        for( int k = 0; k < 4; k++ ) {
            ofDrawLine( to, tc[k] );
            ofDrawLine( tc[k], tc[( k + 1 ) % 4] );
        }
        ofDrawSphere( to, 0.05f );
    }

    // what each camera sees, thin, in its colour; then the merged people, bold
    for( auto& c : mCams ) {
        Camera& cam = *c;
        std::lock_guard< std::mutex > tlk( cam.mtx );
        if( !cam.active() || ( cam.index > 0 && !cam.calibrated ) || ofGetElapsedTimef() - cam.bodiesTime > 0.3 ) continue;
        ofSetColor( optheme::camera( cam.index ), 150 );
        for( auto& b : cam.bodies ) drawBody( b, cam.toRef, 0.012f );
        // the people sent as boxes
        ofSetColor( optheme::boxPerson );
        for( auto& tb : cam.boxes ) {
            glm::vec3 tv[8];
            for( int k = 0; k < 8; k++ ) {
                const glm::vec3 to( k & 1 ? 0.5f : -0.5f, k & 2 ? 0.5f : -0.5f, k & 4 ? 0.5f : -0.5f );
                tv[k] = displayOf( glm::vec3( cam.toRef * glm::vec4( tb.centre + to * tb.size, 1.f )));
            }
            for( int k = 0; k < 8; k++ )
                for( int a : { 1, 2, 4 } ) if( !( k & a )) ofDrawLine( tv[k], tv[k | a] );
        }
    }
    vector< SkeletonFusion::Track > ttracks;
    {
        std::lock_guard< std::mutex > tlk( mFusionMutex );
        ttracks = mFusion.tracks();
    }
    ofSetLineWidth( 3.f );
    for( auto& t : ttracks ) {
        if( ofGetElapsedTimef() - t.lastSeen > 0.5 ) continue;
        ofSetColor( ofColor::fromHsb( ( t.id * 53 ) % 255, 90, 255 ));
        drawBody( t.body, glm::mat4( 1.f ), 0.025f );
    }
    ofSetLineWidth( 1.f );

    ofDisableDepthTest();
    mView.end();

    // ids next to the people
    for( auto& t : ttracks ) {
        if( ofGetElapsedTimef() - t.lastSeen > 0.5 ) continue;
        glm::vec3 tc;
        if( !SkeletonFusion::centreOf( t.body, tc )) continue;
        glm::vec3 ts = mView.worldToScreen( displayOf( tc ), aView );
        ofSetColor( 255 );
        ofDrawBitmapString( "person " + ofToString( t.id ) + ( t.cameras > 1 ? "  (" + ofToString( t.cameras ) + " cameras)" : "" ), ts.x + 10, ts.y );
    }
    ofSetColor( optheme::textMuted );
    ofDrawBitmapString( aTopDown ? "from above (grid 1 m, camera 0 at the bottom)" : "perspective   drag: orbit   wheel: zoom",
                        aView.x + 8, aView.y + 14 );
}

//--------------------------------------------------------------
void ofApp::mousePressed( int x, int y, int ) { mLastMx = x; mLastMy = y; }

void ofApp::mouseDragged( int x, int y, int button ) {
    if( mViewRect.inside( (float)x, (float)y ) && !ImGui::GetIO().WantCaptureMouse && button == OF_MOUSE_BUTTON_LEFT ) {
        mViewAz -= ( x - mLastMx ) * 0.4f;
        mViewEl = glm::clamp( mViewEl + ( y - mLastMy ) * 0.4f, -85.f, 85.f );
    }
    mLastMx = x;
    mLastMy = y;
}

void ofApp::mouseScrolled( int x, int y, float, float scrollY ) {
    if( ImGui::GetIO().WantCaptureMouse || !mViewRect.inside( (float)x, (float)y )) return;
    mViewDist = glm::clamp( mViewDist * std::pow( 0.9f, scrollY ), 0.5f, 40.f );
}

//--------------------------------------------------------------
void ofApp::exit() {
    if( !mDemo ) saveSettings();   // the demo's stand-in cameras are not the rig's settings
    mFinder.stop();
    mOutRunning = false;
    mOutCv.notify_one();
    if( mOutThread.joinable() ) mOutThread.join();
    for( auto& c : mCams ) {
        if( c->connecting.valid() ) c->connecting.wait();
        stopCamera( *c );
        c->recorder.stop();
        if( c->player ) c->player->close();
        c->device.close();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed( int key ) {
    if( ImGui::GetIO().WantTextInput ) return;
    if( key == ' ' ) bSend = !bSend;
    if( key == 'r' ) toggleRecording();
}
