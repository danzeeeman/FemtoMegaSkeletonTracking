#include "ofApp.h"
#include "ofxRtmModelLoader.h"
#include <random>
#include <glm/gtc/matrix_transform.hpp>

namespace {
const ofColor kCameraColors[ofApp::kMaxCameras] = { ofColor( 90, 220, 120 ), ofColor( 255, 160, 60 ), ofColor( 80, 200, 255 ), ofColor( 240, 100, 220 ) };

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
// strings, so an address could not be typed in).
void addParams( ofParameterGroup& aGroup ) {
    for( auto& p : aGroup ) {
        if( auto ts = std::dynamic_pointer_cast< ofParameter< string > >( p )) ofxImGui::AddParameter( *ts );
        else if( auto tb = std::dynamic_pointer_cast< ofParameter< bool > >( p )) ofxImGui::AddParameter( *tb );
        else if( auto ti = std::dynamic_pointer_cast< ofParameter< int > >( p )) ofxImGui::AddParameter( *ti );
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
    if( ofGetEnv( "FEMTO_SELFTEST" ) != "" ) {
        runSelfTest();
        ofExit( 0 );
        return;
    }
    // plain 2D textures: ImGui draws them as GL_TEXTURE_2D
    ofDisableArbTex();
    ofSetFrameRate( 60 );
    ofSetVerticalSync( false );
    ofBackground( 24 );

    gui.setup();

    params.setName( "Output" );
    params.add( oscHost.set( "OSC host", "127.0.0.1" ));
    params.add( oscPort.set( "OSC port", 9999, 1, 65535 ));
    params.add( bSend.set( "Send skeletons", true ));
    params.add( cameraPort.set( "Camera port (all)", 8090, 1, 65535 ));
    // 0 = the camera's fastest mode (highest frame rate, then the smallest image)
    params.add( colorWidth.set( "Colour width (0 fastest)", 0, 0, 3840 ));
    params.add( colorHeight.set( "Colour height (0 fastest)", 0, 0, 2160 ));
    params.add( floorBelow.set( "Floor below camera 0 (m)", 1.5f, 0.f, 5.f ));

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
        // two stand-in cameras, calibrated, watching someone walk about
        for( int i = 0; i < 2; i++ ) {
            mCams[i]->fake = true;
            mCams[i]->K.width = 1280; mCams[i]->K.height = 720; mCams[i]->K.fx = mCams[i]->K.fy = 900.f;
            mCams[i]->leveled = true;
        }
        mCams[1]->toRef = glm::translate( glm::mat4( 1.f ), glm::vec3( 2.4f, 0.f, 4.2f )) * glm::rotate( glm::mat4( 1.f ), glm::radians( 150.f ), glm::vec3( 0.f, 1.f, 0.f ));
        mCams[1]->calibrated = true;
    }
}

//--------------------------------------------------------------
void ofApp::setupCamera( Camera& aCam ) {
    aCam.params.setName( "Camera " + ofToString( aCam.index ));
    aCam.params.add( aCam.enabled.set( "Enabled", aCam.index == 0 ));
    aCam.params.add( aCam.ip.set( "Address", "192.168.1." + ofToString( 10 + aCam.index )));
    aCam.params.add( aCam.levelFromImu.set( "Level from accelerometer", true ));
    aCam.params.add( aCam.invertImu.set( "Accelerometer reads gravity, not up", false ));
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
    // The detector finds the people, the whole-body pose model puts 133 joints on each (body, feet, face, hands).
    // The models are downloaded the first time.
    ofHttpRequest treq;
    treq.timeoutSeconds = 60 * 15;
    ofxRtmPoseTracker::Settings tsettings;
    ofLogNotice( "FemtoSkeletonOsc" ) << "camera " << aCam.index << ": loading models (downloaded on the first run)";
    // the tiny detector: a 416x416 input instead of yolox_m's 640x640, for speed
    tsettings.detectorModelPath = ofxRtmModelLoader::loadModel( "yolox_tiny", "models/RtmDetectors",
        "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/yolox_tiny_8xb8-300e_humanart-6f3252f9.zip", treq );
    tsettings.poseModelPath = ofxRtmModelLoader::loadModel( "rtmpose-l-384x288", "models/RtmPoseWholeBody",
        "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/rtmpose-l_simcc-ucoco_dw-ucoco_270e-384x288-2438fd99_20230728.zip", treq );
    tsettings.logLevel = OF_LOG_NOTICE;
    auto ttracker = std::make_shared< ofxRtmPoseTracker >();
    if( ttracker->setup( tsettings )) {
        ttracker->start();
        aCam.tracker = ttracker;
    } else {
        ofLogError( "FemtoSkeletonOsc" ) << "camera " << aCam.index << ": the pose tracker did not load";
    }
}

//--------------------------------------------------------------
void ofApp::applyOsc() {
    mSkeleton.setup( oscHost, oscPort );
    ofLogNotice( "FemtoSkeletonOsc" ) << "sending OSC to " << oscHost.get() << ":" << oscPort.get();
}

//--------------------------------------------------------------
// Runs on a worker thread: a camera that is off or on another subnet only gives up after a few seconds.
void ofApp::connectCamera( Camera& aCam ) {
    ofxOrbbecDevice::Options topts;
    topts.colorWidth = colorWidth;
    topts.colorHeight = colorHeight;
    // fps and the depth mode left at 0: the fastest the camera offers
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
        size_t tin = 0;
        for( auto& c : mCams ) tin += c->framesIn;
        ofLogNotice( "FemtoSkeletonOsc" ) << "quitting: " << tin << " frames in, " << mBodiesSent << " bodies and "
                                          << mSkeleton.getNumSent() << " OSC messages sent; " << ofGetFrameRate() << " fps";
        ofExit();
        return;
    }

    const double tnow = ofGetElapsedTimef();
    bool tany = false;
    for( auto& c : mCams ) {
        Camera& cam = *c;
        cam.newBodies = false;
        if( !cam.enabled ) {
            if( cam.device.isOpen() && !cam.connecting.valid() ) { cam.device.close(); cam.bodies.clear(); }
            continue;
        }
        // a failed load (no network, say) is a download each time: try again every 30 s, not every frame
        if( !cam.tracker && ofGetElapsedTimef() - cam.lastTrackerTry > 30.f ) {
            cam.lastTrackerTry = ofGetElapsedTimef();
            setupTracker( cam );
        }
        if( cam.connecting.valid() ) {
            if( cam.connecting.wait_for( std::chrono::seconds( 0 )) != std::future_status::ready ) continue;
            if( cam.connecting.get() ) {
                cam.K = cam.device.getColorIntrinsics();
                ofLogNotice( "FemtoSkeletonOsc" ) << "camera " << cam.index << ": Femto " << cam.device.getSerial() << " at " << cam.ip.get()
                                                  << " open, colour " << cam.K.width << "x" << cam.K.height << " fx " << cam.K.fx << " fy " << cam.K.fy;
            }
            cam.lastTry = ofGetElapsedTimef();
        }
        if( !cam.device.isOpen() ) {
            if( ofGetElapsedTimef() - cam.lastTry > 3.f ) connectCamera( cam );
            continue;
        }
        updateCamera( cam );
        tany |= cam.newBodies;
    }

    if( mDemo ) { updateDemo(); tany = true; }
    for( int i = 1; i < kMaxCameras; i++ ) if( mCams[i]->collecting && mCams[i]->newBodies ) collectCalibration( *mCams[i] );

    if( !tany ) return;

    // everything the calibrated cameras saw lately, in camera 0's frame, merged into people
    std::vector< SkeletonFusion::Detection > tdets;
    for( auto& c : mCams ) {
        const Camera& cam = *c;
        if( !cam.active() || ( cam.index > 0 && !cam.calibrated )) continue;
        if( tnow - cam.bodiesTime > 0.3 ) continue;
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
    for( auto& t : mFusion.update( tdets, tnow )) {
        if( !t.updated ) continue;
        FemtoSkeleton::Body tb = t.body;
        if( !mSkeleton.toSensorFrame( tb )) continue;
        if( bSend ) {
            mSkeleton.send( tb );
            mBodiesSent++;
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
        cam.bodies.assign( 1, tb );
        cam.bodiesTime = ofGetElapsedTimef();
        cam.newBodies = true;
    }
}

//--------------------------------------------------------------
void ofApp::updateCamera( Camera& aCam ) {
    aCam.device.update();
    const ofxOrbbecFrame& tframe = aCam.device.getFrame();
    if( tframe.isNew && tframe.color.isAllocated() ) {
        aCam.framesIn++;
        aCam.color = tframe.color;
        aCam.depth = tframe.depth;
        if( aCam.tex.isAllocated() && ( aCam.tex.getWidth() != aCam.color.getWidth() || aCam.tex.getHeight() != aCam.color.getHeight() )) aCam.tex.clear();
        aCam.tex.loadData( aCam.color );

        if( aCam.tracker ) {
            // keypoints come back as pixels of the colour image
            aCam.tracker->setOutRect( ofRectangle( 0, 0, aCam.color.getWidth(), aCam.color.getHeight() ));
            if( aCam.tracker->run( aCam.color )) aCam.toTracker++;
        }
    }

    // the camera's pitch and roll: from its accelerometer when it has one
    glm::vec3 tup;
    glm::mat3 tlevel;
    aCam.leveled = false;
    if( aCam.levelFromImu && aCam.device.getUp( tup )) {
        if( aCam.invertImu ) tup = -tup;
        aCam.pitchDeg = glm::degrees( std::asin( glm::clamp( tup.z, -1.f, 1.f )));
        aCam.rollDeg = glm::degrees( std::atan2( tup.x, -tup.y ));
        if( levelMatrix( tup, tlevel )) {
            mSkeleton.setLevelMatrix( tlevel );
            aCam.levelM = tlevel;
            aCam.leveled = true;
        }
    }
    if( !aCam.leveled ) mSkeleton.clearLevelMatrix();

    // new poses: build the skeletons
    if( aCam.tracker && aCam.tracker->hasNewData() ) {
        aCam.bodies.clear();
        // debug-build counts, logged once a second
        static int sPoses = 0, sFound = 0, sScored = 0, sWithDepth = 0, sBuilt = 0, sUpdates = 0;
        static float sScoreMax = 0.f, sLastLog = 0.f;
        for( auto& tpose : aCam.tracker->getPoses() ) {
            if( mDebug && tpose ) {
                sPoses++;
                if( tpose->bFoundThisFrame ) {
                    sFound++;
                    // the body keypoints build() uses: COCO 5..16
                    for( int k = 5; k <= 16 && k < (int)tpose->keypoints.size() && k < (int)tpose->scores.size(); k++ ) {
                        sScoreMax = std::max( sScoreMax, tpose->scores[k] );
                        if( tpose->scores[k] < mSkeleton.minScore ) continue;
                        sScored++;
                        ofVec3f tp;
                        if( mSkeleton.unproject( ofVec2f( tpose->keypoints[k].x, tpose->keypoints[k].y ), aCam.depth, aCam.K, tp )) sWithDepth++;
                    }
                }
            }
            if( !tpose || !tpose->bFoundThisFrame ) continue;
            FemtoSkeleton::Body tbody;
            if( mSkeleton.build( *tpose, aCam.depth, aCam.K, tbody )) aCam.bodies.push_back( tbody );
        }
        if( mDebug ) {
            sUpdates++;
            sBuilt += (int)aCam.bodies.size();
            if( ofGetElapsedTimef() - sLastLog > 1.f ) {
                size_t tvalid = 0;
                const size_t tnpix = (size_t)aCam.depth.getWidth() * aCam.depth.getHeight();
                for( size_t i = 0; i < tnpix; i++ ) if( aCam.depth.getData()[i] ) tvalid++;
                ofLogNotice( "FemtoSkeletonOsc" ) << "debug cam " << aCam.index << ": " << sUpdates << " tracker updates, " << sPoses << " poses, "
                    << sFound << " found, body keypoints scored " << sScored << " (max score " << sScoreMax << "), with depth " << sWithDepth
                    << ", bodies built " << sBuilt << " | depth " << aCam.depth.getWidth() << "x" << aCam.depth.getHeight() << " "
                    << ( tnpix ? 100.0 * tvalid / tnpix : 0.0 ) << "% valid, colour " << aCam.color.getWidth() << "x" << aCam.color.getHeight()
                    << ", K " << aCam.K.width << "x" << aCam.K.height;
                glm::vec3 traw;
                if( aCam.device.getUp( traw )) {
                    ofLogNotice( "FemtoSkeletonOsc" ) << "debug cam " << aCam.index << ": accelerometer up (colour camera frame, y down) "
                        << traw.x << " " << traw.y << " " << traw.z << ( aCam.invertImu ? ", inverted by the setting" : "" )
                        << ", pitch " << aCam.pitchDeg << " roll " << aCam.rollDeg << ( aCam.leveled ? " levelling" : "" );
                }
                sPoses = sFound = sScored = sWithDepth = sBuilt = sUpdates = 0;
                sScoreMax = 0.f;
                sLastLog = ofGetElapsedTimef();
            }
        }
        aCam.bodiesTime = ofGetElapsedTimef();
        aCam.newBodies = true;
    }
    mSkeleton.clearLevelMatrix();
}

//--------------------------------------------------------------
// While collecting: one person alone in view of both this camera and camera 0, at about the same moment, gives
// a point for every joint both place.
void ofApp::collectCalibration( Camera& aCam ) {
    const Camera& tref = *mCams[0];
    if( !tref.device.isOpen() || tref.bodies.size() != 1 || aCam.bodies.size() != 1 ) return;
    if( std::abs( tref.bodiesTime - aCam.bodiesTime ) > 0.1 ) return;
    const auto& ta = aCam.bodies[0];
    const auto& tb = tref.bodies[0];
    for( int j = 0; j < FemtoSkeleton::TOTAL_JOINTS; j++ )
        if( ta.seen[j] && tb.seen[j] ) aCam.cal.add( vec3Of( ta.cam[j] ), vec3Of( tb.cam[j] ));
}

//--------------------------------------------------------------
void ofApp::draw() {
    std::vector< Camera* > tshown;
    for( auto& c : mCams ) if( c->enabled && c->tex.isAllocated() ) tshown.push_back( c.get() );

    // the right of the window: each camera with its poses over it, in a column, and the joined space in 3D
    ofRectangle tfull( 450, 10, ofGetWidth() - 460, ofGetHeight() - 20 );
    const bool tcams = !tshown.empty();
    ofRectangle tarea( tfull.x, tfull.y, tcams ? tfull.width * 0.36f : 0.f, tfull.height );
    mViewRect = ofRectangle( tarea.getRight() + ( tcams ? 6.f : 0.f ), tfull.y, tfull.getRight() - tarea.getRight() - ( tcams ? 6.f : 0.f ), tfull.height );
    int tcols = 1;
    int trows = std::max( 1, (int)tshown.size() );
    float tw = tarea.width / tcols, th = tarea.height / trows;
    for( size_t i = 0; i < tshown.size(); i++ ) {
        Camera& cam = *tshown[i];
        ofRectangle ttile( tarea.x + ( i % tcols ) * tw, tarea.y + ( i / tcols ) * th, tw - 6, th - 6 );
        ofRectangle timg( 0, 0, cam.tex.getWidth(), cam.tex.getHeight() );
        timg.scaleTo( ttile, OF_SCALEMODE_FIT );
        timg.setPosition( ttile.x, ttile.y );
        ofSetColor( 255 );
        cam.tex.draw( timg );
        if( cam.tracker ) cam.tracker->draw( timg );
        ofSetColor( kCameraColors[cam.index] );
        ofDrawBitmapString( "camera " + ofToString( cam.index ) + ( cam.index > 0 && !cam.calibrated ? "  (not calibrated)" : "" ), timg.x + 6, timg.y + 14 );
    }

    draw3d( mViewRect );

    gui.begin();
    guiSettings.windowPos = ofVec2f( 10, 10 );
    guiSettings.windowSize = ofVec2f( 430, ofGetHeight() - 20 );
    ofxImGui::BeginWindow( "Femto Skeleton OSC", guiSettings, ImGuiWindowFlags_NoCollapse );

    int tfused = 0;
    for( auto& t : mFusion.tracks() ) if( ofGetElapsedTimef() - t.lastSeen < 0.5 ) tfused++;
    ImGui::Text( "people now %d   sent %zu   OSC messages %zu   app %.0f fps", tfused, mBodiesSent, mSkeleton.getNumSent(), ofGetFrameRate() );
    ImGui::Separator();
    if( ImGui::CollapsingHeader( params.getName().c_str(), ImGuiTreeNodeFlags_DefaultOpen )) addParams( params );
    if( ImGui::Button( "Apply OSC address" )) applyOsc();
    ImGui::SameLine();
    if( ImGui::Button( "Floor from feet" )) floorFromFeet();
    ImGui::SameLine();
    if( ImGui::Button( "Save settings" )) saveSettings();
    ImGui::Separator();
    for( auto& c : mCams ) drawCameraPanel( *c );
    ImGui::Separator();
    ofxImGui::AddGroup( mFusion.params, guiSettings );
    ofxImGui::AddGroup( mSkeleton.getParams(), guiSettings );
    if( mCams[0]->tracker ) {
        ImGui::Separator();
        ofxImGui::AddGroup( mCams[0]->tracker->getParams( "Pose tracker" ), guiSettings );
    }
    ofxImGui::EndWindow( guiSettings );
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
void ofApp::drawCameraPanel( Camera& aCam ) {
    ImGui::PushID( aCam.index );
    const bool topen = aCam.enabled || aCam.index == 0;
    if( ImGui::CollapsingHeader( ( "Camera " + ofToString( aCam.index ) + ( aCam.index == 0 ? " (reference)" : "" )).c_str(), topen ? ImGuiTreeNodeFlags_DefaultOpen : 0 )) {
        addParams( aCam.params );
        if( aCam.device.isOpen() ) {
            ImGui::Text( "%s  %dx%d  frames %zu  to tracker %zu  %.0f fps", aCam.device.getSerial().c_str(), aCam.K.width, aCam.K.height,
                         aCam.framesIn, aCam.toTracker, aCam.tracker ? aCam.tracker->getFps() : 0.0 );
            if( aCam.device.hasImu() ) {
                ImGui::Text( "accelerometer: pitch %.1f  roll %.1f degrees%s", aCam.pitchDeg, aCam.rollDeg, aCam.leveled ? "  (levelling)" : "" );
            } else {
                ImGui::TextColored( ImVec4( 1.f, 0.8f, 0.3f, 1.f ), "no accelerometer reading: not levelled" );
            }
            ImGui::Text( "bodies now %zu", aCam.bodies.size() );
        } else if( aCam.enabled ) {
            if( aCam.connecting.valid() ) ImGui::TextColored( ImVec4( 1.f, 0.8f, 0.3f, 1.f ), "connecting to %s ...", aCam.ip.get().c_str() );
            else ImGui::TextColored( ImVec4( 1.f, 0.5f, 0.3f, 1.f ), "no camera at %s", aCam.ip.get().c_str() );
        }

        if( aCam.index > 0 ) {
            ImGui::Separator();
            // levelled by the accelerometer, both cameras differ only by a turn about the vertical and a shift
            const bool tyaw = aCam.leveled && mCams[0]->leveled;
            if( aCam.calibrated ) {
                ImGui::Text( "calibrated: rms %.1f cm over %d points", aCam.calRms * 100.f, aCam.calInliers );
            } else {
                ImGui::TextColored( ImVec4( 1.f, 0.5f, 0.3f, 1.f ), "not calibrated: its people are not sent" );
            }
            ImGui::Checkbox( "Collect points", &aCam.collecting );
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
            if( ImGui::Button( "Forget calibration" )) {
                aCam.calibrated = false;
                aCam.toRef = glm::mat4( 1.f );
                saveCalibration();
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
    for( auto& t : mFusion.tracks() ) {
        if( ofGetElapsedTimef() - t.lastSeen > 0.5 ) continue;
        for( int j : { (int)FemtoSkeleton::FOOT_LEFT, (int)FemtoSkeleton::FOOT_RIGHT, (int)FemtoSkeleton::ANKLE_LEFT, (int)FemtoSkeleton::ANKLE_RIGHT } )
            if( t.body.seen[j] ) tmax = std::max( tmax, t.body.cam[j].y );
    }
    if( tmax > 0.f ) floorBelow = tmax;
}

//--------------------------------------------------------------
void ofApp::draw3d( const ofRectangle& aView ) {
    if( aView.width < 20 || aView.height < 20 ) return;
    ofSetColor( 14, 15, 20 );
    ofDrawRectangle( aView );

    const float taz = glm::radians( mViewAz ), tel = glm::radians( mViewEl );
    mView.setNearClip( 0.05f );
    mView.setFarClip( 100.f );
    mView.setFov( 45.f );
    mView.setPosition( mViewTarget + mViewDist * glm::vec3( std::cos( tel ) * std::cos( taz ), std::cos( tel ) * std::sin( taz ), std::sin( tel )));
    mView.lookAt( mViewTarget, glm::vec3( 0.f, 0.f, 1.f ));

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
    ofSetColor( 60, 64, 80 );
    for( int i = -6; i <= 6; i++ ) {
        ofDrawLine( (float)i, -1.f, tfz, (float)i, 11.f, tfz );
        ofDrawLine( -6.f, i + 5.f, tfz, 6.f, i + 5.f, tfz );
    }
    // camera 0's axes: x red, forward green, up blue
    ofSetLineWidth( 2.f );
    ofSetColor( 255, 80, 80 );  ofDrawLine( 0.f, 0.f, 0.f, 0.5f, 0.f, 0.f );
    ofSetColor( 80, 255, 80 );  ofDrawLine( 0.f, 0.f, 0.f, 0.f, 0.5f, 0.f );
    ofSetColor( 80, 140, 255 ); ofDrawLine( 0.f, 0.f, 0.f, 0.f, 0.f, 0.5f );
    ofSetLineWidth( 1.f );

    // the cameras: a frustum each, where it is in the joined space, in its own colour
    for( auto& c : mCams ) {
        const Camera& cam = *c;
        if( !cam.active() || ( cam.index > 0 && !cam.calibrated ) || cam.K.fx <= 0.f ) continue;
        const glm::mat3 tm = cam.leveled ? cam.levelM : glm::mat3( 1.f );
        auto corner = [&]( float sx, float sy ) {
            const float td = 0.6f;
            glm::vec3 p( sx * ( cam.K.width * 0.5f ) / cam.K.fx * td, sy * ( cam.K.height * 0.5f ) / cam.K.fy * td, td );
            return displayOf( glm::vec3( cam.toRef * glm::vec4( tm * p, 1.f )));
        };
        const glm::vec3 to = displayOf( glm::vec3( cam.toRef * glm::vec4( 0.f, 0.f, 0.f, 1.f )));
        const glm::vec3 tc[4] = { corner( -1.f, -1.f ), corner( 1.f, -1.f ), corner( 1.f, 1.f ), corner( -1.f, 1.f ) };
        ofSetColor( kCameraColors[cam.index] );
        for( int k = 0; k < 4; k++ ) {
            ofDrawLine( to, tc[k] );
            ofDrawLine( tc[k], tc[( k + 1 ) % 4] );
        }
        ofDrawSphere( to, 0.05f );
    }

    // what each camera sees, thin, in its colour; then the merged people, bold
    for( auto& c : mCams ) {
        const Camera& cam = *c;
        if( !cam.active() || ( cam.index > 0 && !cam.calibrated ) || ofGetElapsedTimef() - cam.bodiesTime > 0.3 ) continue;
        ofSetColor( kCameraColors[cam.index], 150 );
        for( auto& b : cam.bodies ) drawBody( b, cam.toRef, 0.012f );
    }
    ofSetLineWidth( 3.f );
    for( auto& t : mFusion.tracks() ) {
        if( ofGetElapsedTimef() - t.lastSeen > 0.5 ) continue;
        ofSetColor( ofColor::fromHsb( ( t.id * 53 ) % 255, 90, 255 ));
        drawBody( t.body, glm::mat4( 1.f ), 0.025f );
    }
    ofSetLineWidth( 1.f );

    ofDisableDepthTest();
    mView.end();

    // ids next to the people
    for( auto& t : mFusion.tracks() ) {
        if( ofGetElapsedTimef() - t.lastSeen > 0.5 ) continue;
        glm::vec3 tc;
        if( !SkeletonFusion::centreOf( t.body, tc )) continue;
        glm::vec3 ts = mView.worldToScreen( displayOf( tc ), aView );
        ofSetColor( 255 );
        ofDrawBitmapString( "person " + ofToString( t.id ) + ( t.cameras > 1 ? "  (" + ofToString( t.cameras ) + " cameras)" : "" ), ts.x + 10, ts.y );
    }
    ofSetColor( 170 );
    ofDrawBitmapString( "the joined space, in camera 0's frame   drag: orbit   wheel: zoom", aView.x + 8, aView.y + 14 );
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
    saveSettings();
    for( auto& c : mCams ) {
        if( c->connecting.valid() ) c->connecting.wait();
        if( c->tracker ) c->tracker->stop();
        c->device.close();
    }
}

//--------------------------------------------------------------
void ofApp::keyPressed( int key ) {
    if( ImGui::GetIO().WantTextInput ) return;
    if( key == ' ' ) bSend = !bSend;
}
