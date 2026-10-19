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
    params.add( colorWidth.set( "Colour width", 1280, 320, 3840 ));
    params.add( colorHeight.set( "Colour height", 720, 240, 2160 ));

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

    mAutoQuitSec = ofToFloat( ofGetEnv( "TRACKER_QUIT_AFTER" ));
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
    tsettings.detectorModelPath = ofxRtmModelLoader::loadModel( "yolox_m", "models/RtmDetectors",
        "https://download.openmmlab.com/mmpose/v1/projects/rtmposev1/onnx_sdk/yolox_m_8xb8-300e_humanart-c2c7a14a.zip", treq );
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
    topts.colorFps = 30;
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
        if( !cam.tracker ) setupTracker( cam );
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

    for( int i = 1; i < kMaxCameras; i++ ) if( mCams[i]->collecting && mCams[i]->newBodies ) collectCalibration( *mCams[i] );

    if( !tany ) return;

    // everything the calibrated cameras saw lately, in camera 0's frame, merged into people
    std::vector< SkeletonFusion::Detection > tdets;
    for( auto& c : mCams ) {
        const Camera& cam = *c;
        if( !cam.device.isOpen() || ( cam.index > 0 && !cam.calibrated )) continue;
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
            aCam.leveled = true;
        }
    }
    if( !aCam.leveled ) mSkeleton.clearLevelMatrix();

    // new poses: build the skeletons
    if( aCam.tracker && aCam.tracker->hasNewData() ) {
        aCam.bodies.clear();
        for( auto& tpose : aCam.tracker->getPoses() ) {
            if( !tpose || !tpose->bFoundThisFrame ) continue;
            FemtoSkeleton::Body tbody;
            if( mSkeleton.build( *tpose, aCam.depth, aCam.K, tbody )) aCam.bodies.push_back( tbody );
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

    // the right of the window: each camera with its poses over it, and a plan view of everyone
    ofRectangle tarea( 450, 10, ofGetWidth() - 460, ofGetHeight() - 20 );
    int tcols = tshown.size() <= 1 ? 1 : 2;
    int trows = std::max( 1, (int)( ( tshown.size() + tcols - 1 ) / tcols ));
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

    // plan view, in camera 0's frame: x across, z away from camera 0, 6 m deep. Each camera's own detections in its
    // colour, the merged people in white rings, the cameras as triangles: after calibration they lie on each other.
    float tpw = 300, tph = 300;
    ofRectangle tplan( tarea.getRight() - tpw - 6, tarea.getBottom() - tph - 6, tpw, tph );
    ofSetColor( 0, 190 );
    ofDrawRectangle( tplan );
    ofSetColor( 255, 60 );
    ofDrawLine( tplan.getCenter().x, tplan.y, tplan.getCenter().x, tplan.getBottom() );
    const float tscale = tph / 6.f;
    auto plan = [&]( const glm::vec3& p ) { return glm::vec2( tplan.getCenter().x + p.x * tscale, tplan.y + p.z * tscale ); };
    for( auto& c : mCams ) {
        const Camera& cam = *c;
        if( !cam.device.isOpen() || ( cam.index > 0 && !cam.calibrated )) continue;
        ofSetColor( kCameraColors[cam.index] );
        glm::vec2 tc = plan( glm::vec3( cam.toRef * glm::vec4( 0.f, 0.f, 0.f, 1.f )));
        ofDrawTriangle( tc.x, tc.y - 5, tc.x - 4, tc.y + 3, tc.x + 4, tc.y + 3 );
        for( auto& b : cam.bodies )
            for( int j = 0; j < FemtoSkeleton::TOTAL_JOINTS; j++ ) {
                if( !b.seen[j] ) continue;
                glm::vec2 tp = plan( glm::vec3( cam.toRef * glm::vec4( vec3Of( b.cam[j] ), 1.f )));
                ofDrawCircle( tp.x, tp.y, 1.5f );
            }
    }
    for( auto& t : mFusion.tracks() ) {
        if( ofGetElapsedTimef() - t.lastSeen > 0.5 ) continue;
        glm::vec3 tc;
        if( !SkeletonFusion::centreOf( t.body, tc )) continue;
        glm::vec2 tp = plan( tc );
        ofNoFill();
        ofSetColor( 255 );
        ofDrawCircle( tp.x, tp.y, 7.f );
        ofFill();
        ofDrawBitmapString( ofToString( t.id ), tp.x + 9, tp.y + 4 );
    }

    gui.begin();
    guiSettings.windowPos = ofVec2f( 10, 10 );
    guiSettings.windowSize = ofVec2f( 430, ofGetHeight() - 20 );
    ofxImGui::BeginWindow( "Femto Skeleton OSC", guiSettings, ImGuiWindowFlags_NoCollapse );

    int tfused = 0;
    for( auto& t : mFusion.tracks() ) if( ofGetElapsedTimef() - t.lastSeen < 0.5 ) tfused++;
    ImGui::Text( "people now %d   sent %zu   OSC messages %zu   app %.0f fps", tfused, mBodiesSent, mSkeleton.getNumSent(), ofGetFrameRate() );
    ImGui::Separator();
    ofxImGui::AddGroup( params, guiSettings );
    if( ImGui::Button( "Apply OSC address" )) applyOsc();
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
}

//--------------------------------------------------------------
void ofApp::drawCameraPanel( Camera& aCam ) {
    ImGui::PushID( aCam.index );
    const bool topen = aCam.enabled || aCam.index == 0;
    if( ImGui::CollapsingHeader( ( "Camera " + ofToString( aCam.index ) + ( aCam.index == 0 ? " (reference)" : "" )).c_str(), topen ? ImGuiTreeNodeFlags_DefaultOpen : 0 )) {
        ofxImGui::AddGroup( aCam.params, guiSettings );
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
