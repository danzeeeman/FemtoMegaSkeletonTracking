//
//  FemtoSkeleton.h
//  TrackerFemto
//
//  Turns the poses RTMPose finds in the Femto Mega's colour image into the skeletons Mimic gets from the
//  tracker, and sends them over OSC the way the tracker's OSCSkeletonSender did:
//      /bodies/<id>/joints/<joint>   x y z "Tracked" | "NotTracked"
//
//  Each keypoint is a pixel in the colour image. The depth image is registered to the colour camera
//  (ofxOrbbecDevice::Options::alignDepthToColor), so the same pixel reads the depth, and the camera's
//  intrinsics put it in 3D. What goes out is in the Kinect's frame, which is what the listener expects:
//  metres, y up, and z negative away from the sensor (the listener negates it on receipt).
//
//  RTMPose finds COCO-17 joints (and hands, with the whole-body model); the Kinect's skeleton has 25, so
//  the spine, neck, head and hand joints are made from the ones found.
//

#pragma once
#include "ofMain.h"
#include "ofxOsc.h"
#include "ofxOrbbecTypes.h"
#include "ofxRtmPoseTracker.h"
#include <array>
#include <algorithm>

class FemtoSkeleton {
public:
    // The order of Skeleton::JointIndex in Mimic.
    enum Joint {
        SPINE_BASE = 0, SPINE_MID, SPINE_SHOULDER, NECK, HEAD,
        SHOULDER_LEFT, ELBOW_LEFT, WRIST_LEFT, HAND_LEFT, HAND_TIP_LEFT, THUMB_LEFT,
        SHOULDER_RIGHT, ELBOW_RIGHT, WRIST_RIGHT, HAND_RIGHT, HAND_TIP_RIGHT, THUMB_RIGHT,
        HIP_LEFT, KNEE_LEFT, ANKLE_LEFT, FOOT_LEFT,
        HIP_RIGHT, KNEE_RIGHT, ANKLE_RIGHT, FOOT_RIGHT,
        TOTAL_JOINTS
    };

    static const char* jointName( int aJoint ) {
        static const char* const tnames[TOTAL_JOINTS] = {
            "SpineBase", "SpineMid", "SpineShoulder", "Neck", "Head",
            "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft", "HandTipLeft", "ThumbLeft",
            "ShoulderRight", "ElbowRight", "WristRight", "HandRight", "HandTipRight", "ThumbRight",
            "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft",
            "HipRight", "KneeRight", "AnkleRight", "FootRight" };
        return tnames[aJoint];
    }

    // A body in the sensor's frame (metres, as Mimic's listener receives it).
    struct Body {
        uint64_t id = 0;
        std::array< ofVec3f, TOTAL_JOINTS > pos;
        std::array< bool, TOTAL_JOINTS > seen;
        // the 3D joints in the camera's own frame (x right, y down, z forward), for drawing
        std::array< ofVec3f, TOTAL_JOINTS > cam;
        int numSeen = 0;
        Body() { seen.fill( false ); }
    };

    //--------------------------------------------------------------
    void setup( const string& aHost, int aPort ) {
        mSender.setup( aHost, aPort );
        mHost = aHost;
        mPort = aPort;
    }

    ofParameterGroup& getParams() {
        if( mParams.size() == 0 ) {
            mParams.setName( "skeleton" );
            mParams.add( minScore.set( "Min keypoint score", 0.35f, 0.f, 1.f ));
            mParams.add( depthWindow.set( "Depth window (px)", 5, 1, 15 ));
            mParams.add( minDepthM.set( "Min depth (m)", 0.3f, 0.1f, 2.f ));
            mParams.add( maxDepthM.set( "Max depth (m)", 5.f, 1.f, 10.f ));
            mParams.add( pitchDeg.set( "Camera pitch (deg)", 0.f, -90.f, 90.f ));
            mParams.add( rollDeg.set( "Camera roll (deg)", 0.f, -90.f, 90.f ));
            mParams.add( flipX.set( "Flip X (Kinect x is the sensor's left)", true ));
            mParams.add( minJoints.set( "Joints to send a body", 6, 1, 25 ));
        }
        return mParams;
    }

    //--------------------------------------------------------------
    // The 3D point under a colour pixel, in the camera frame (metres). False with no good depth there.
    bool unproject( const ofVec2f& aPix, const ofShortPixels& aDepth, const ofxOrbbecIntrinsics& aK, ofVec3f& aOut ) const {
        int tw = aDepth.getWidth(), th = aDepth.getHeight();
        if( tw < 1 || th < 1 || aK.fx <= 0.f ) return false;
        int tu = (int)roundf( aPix.x ), tv = (int)roundf( aPix.y );
        if( tu < 0 || tv < 0 || tu >= tw || tv >= th ) return false;
        // the depth picture is as big as the colour one it is registered to, but not always what the
        // intrinsics were read from: scale to it
        float tsx = aK.width > 0 ? (float)tw / aK.width : 1.f;
        float tsy = aK.height > 0 ? (float)th / aK.height : 1.f;

        // the median of the valid depths in a window, so a hole or an edge in the depth does not spoil a joint
        int thalf = depthWindow / 2;
        float tvals[225];
        int tn = 0;
        const unsigned short* tdata = aDepth.getData();
        for( int dy = -thalf; dy <= thalf; dy++ ) {
            int ty = tv + dy;
            if( ty < 0 || ty >= th ) continue;
            for( int dx = -thalf; dx <= thalf; dx++ ) {
                int tx = tu + dx;
                if( tx < 0 || tx >= tw ) continue;
                unsigned short td = tdata[ty * tw + tx];
                if( td == 0 ) continue;
                float tm = td * 0.001f;
                if( tm < minDepthM || tm > maxDepthM ) continue;
                if( tn < 225 ) tvals[tn++] = tm;
            }
        }
        if( tn < 1 ) return false;
        std::nth_element( tvals, tvals + tn / 2, tvals + tn );
        float tz = tvals[tn / 2];
        aOut.x = ( aPix.x - aK.cx * tsx ) * tz / ( aK.fx * tsx );
        aOut.y = ( aPix.y - aK.cy * tsy ) * tz / ( aK.fy * tsy );
        aOut.z = tz;
        return true;
    }

    //--------------------------------------------------------------
    // The camera hangs tilted, so its y is not up. Pitch (about the camera's x) and roll (about its z), in degrees,
    // level the points; adjust them until the floor and the people stand straight in the plan and in Mimic.
    ofVec3f level( const ofVec3f& aCam ) const {
        if( mUseLevelMatrix ) {
            glm::vec3 tl = mLevelMatrix * glm::vec3( aCam.x, aCam.y, aCam.z );
            return ofVec3f( tl.x, tl.y, tl.z );
        }
        float tp = ofDegToRad( pitchDeg ), tr = ofDegToRad( rollDeg );
        float tcp = cosf( tp ), tsp = sinf( tp ), tcr = cosf( tr ), tsr = sinf( tr );
        ofVec3f t( aCam.x, aCam.y * tcp - aCam.z * tsp, aCam.y * tsp + aCam.z * tcp );
        return ofVec3f( t.x * tcr - t.y * tsr, t.x * tsr + t.y * tcr, t.z );
    }

    //--------------------------------------------------------------
    // The camera's pitch and roll from its accelerometer instead of the two sliders: a matrix that turns a point in
    // the camera's frame into the same point in a frame with y straight down and z the way the camera faces,
    // flattened onto the horizon. Used by build() until cleared.
    void setLevelMatrix( const glm::mat3& aMatrix ) { mLevelMatrix = aMatrix; mUseLevelMatrix = true; }
    void clearLevelMatrix() { mUseLevelMatrix = false; }

    //--------------------------------------------------------------
    // False when too few joints could be placed in 3D to be worth sending.
    bool build( ofxRtmTrackedPose& aPose, const ofShortPixels& aDepth, const ofxOrbbecIntrinsics& aK, Body& aBody ) const {
        aBody = Body();
        aBody.id = aPose.ID;

        const int tnum = (int)aPose.keypoints.size();
        const bool twhole = tnum >= 133;
        // a keypoint's 3D place, or not
        auto get = [&]( int aIndex, ofVec3f& aOut ) -> bool {
            if( aIndex < 0 || aIndex >= tnum ) return false;
            if( aPose.scores.size() <= (size_t)aIndex || aPose.scores[aIndex] < minScore ) return false;
            const glm::vec2& tk = aPose.keypoints[aIndex];
            if( !ofxRtmPose::sIsKeypointValid( tk )) return false;
            if( !unproject( ofVec2f( tk.x, tk.y ), aDepth, aK, aOut )) return false;
            aOut = level( aOut );
            return true;
        };
        auto set = [&]( int aJoint, const ofVec3f& aCam ) {
            aBody.cam[aJoint] = aCam;
            aBody.seen[aJoint] = true;
        };
        auto direct = [&]( int aJoint, int aKeypoint ) {
            ofVec3f tp;
            if( get( aKeypoint, tp )) set( aJoint, tp );
        };

        // COCO-17 indices: 0 nose, 3/4 ears, 5/6 shoulders, 7/8 elbows, 9/10 wrists, 11/12 hips, 13/14 knees, 15/16 ankles
        direct( SHOULDER_LEFT, 5 );  direct( SHOULDER_RIGHT, 6 );
        direct( ELBOW_LEFT, 7 );     direct( ELBOW_RIGHT, 8 );
        direct( WRIST_LEFT, 9 );     direct( WRIST_RIGHT, 10 );
        direct( HIP_LEFT, 11 );      direct( HIP_RIGHT, 12 );
        direct( KNEE_LEFT, 13 );     direct( KNEE_RIGHT, 14 );
        direct( ANKLE_LEFT, 15 );    direct( ANKLE_RIGHT, 16 );

        // feet: the big toe in the whole-body model, else the ankle
        if( twhole ) {
            direct( FOOT_LEFT, 17 );
            direct( FOOT_RIGHT, 20 );
        }
        if( !aBody.seen[FOOT_LEFT] && aBody.seen[ANKLE_LEFT] ) set( FOOT_LEFT, aBody.cam[ANKLE_LEFT] );
        if( !aBody.seen[FOOT_RIGHT] && aBody.seen[ANKLE_RIGHT] ) set( FOOT_RIGHT, aBody.cam[ANKLE_RIGHT] );

        // hands: the whole-body model has them. Hand is the middle finger's knuckle, the tip its end.
        // left hand 91..111 (wrist, thumb 1-4, index 1-4, middle 1-4...), right hand 112..132
        for( int tside = 0; tside < 2; tside++ ) {
            int tbase = tside == 0 ? 91 : 112;
            int tHand = tside == 0 ? HAND_LEFT : HAND_RIGHT;
            int tTip = tside == 0 ? HAND_TIP_LEFT : HAND_TIP_RIGHT;
            int tThumb = tside == 0 ? THUMB_LEFT : THUMB_RIGHT;
            int tWrist = tside == 0 ? WRIST_LEFT : WRIST_RIGHT;
            if( twhole ) {
                direct( tHand, tbase + 9 );
                direct( tTip, tbase + 12 );
                direct( tThumb, tbase + 4 );
            }
            // no hand found: the hand is at the wrist, so the arm still ends somewhere. Not tracked beyond it.
            if( !aBody.seen[tHand] && aBody.seen[tWrist] ) set( tHand, aBody.cam[tWrist] );
        }

        // the spine and head are made from what was found
        ofVec3f tl, tr;
        bool tsh = aBody.seen[SHOULDER_LEFT] && aBody.seen[SHOULDER_RIGHT];
        bool thp = aBody.seen[HIP_LEFT] && aBody.seen[HIP_RIGHT];
        if( tsh ) set( SPINE_SHOULDER, ( aBody.cam[SHOULDER_LEFT] + aBody.cam[SHOULDER_RIGHT] ) * 0.5f );
        if( thp ) set( SPINE_BASE, ( aBody.cam[HIP_LEFT] + aBody.cam[HIP_RIGHT] ) * 0.5f );
        if( tsh && thp ) set( SPINE_MID, ( aBody.cam[SPINE_SHOULDER] + aBody.cam[SPINE_BASE] ) * 0.5f );

        // the head: halfway between the ears, else the nose
        ofVec3f thead;
        bool theadOk = false;
        if( get( 3, tl ) && get( 4, tr )) {
            thead = ( tl + tr ) * 0.5f;
            theadOk = true;
        } else if( get( 0, tl )) {
            thead = tl;
            theadOk = true;
        }
        if( theadOk ) set( HEAD, thead );
        // the neck is a little way up from the shoulders to the head
        if( tsh && theadOk ) set( NECK, aBody.cam[SPINE_SHOULDER] + ( thead - aBody.cam[SPINE_SHOULDER] ) * 0.3f );

        // to the Kinect's frame
        for( int j = 0; j < TOTAL_JOINTS; j++ ) {
            if( !aBody.seen[j] ) continue;
            const ofVec3f& tc = aBody.cam[j];
            // the camera has y down and z forward; the Kinect's y is up, and Mimic's listener wants z away from
            // the sensor to be negative. The Kinect's x runs to the sensor's left.
            aBody.pos[j] = ofVec3f( flipX ? -tc.x : tc.x, -tc.y, -tc.z );
            aBody.numSeen++;
        }
        return aBody.numSeen >= minJoints;
    }

    //--------------------------------------------------------------
    // A body whose `cam` joints are set (by build(), or by merging several cameras' bodies): fills in `pos`, the
    // Kinect-frame joints that send() sends, and `numSeen`. False when too few joints are seen.
    bool toSensorFrame( Body& aBody ) const {
        aBody.numSeen = 0;
        for( int j = 0; j < TOTAL_JOINTS; j++ ) {
            if( !aBody.seen[j] ) continue;
            const ofVec3f& tc = aBody.cam[j];
            aBody.pos[j] = ofVec3f( flipX ? -tc.x : tc.x, -tc.y, -tc.z );
            aBody.numSeen++;
        }
        return aBody.numSeen >= minJoints;
    }

    //--------------------------------------------------------------
    void send( const Body& aBody ) {
        string tid = ofToString( aBody.id );
        for( int j = 0; j < TOTAL_JOINTS; j++ ) {
            ofxOscMessage tmsg;
            tmsg.setAddress( "/bodies/" + tid + "/joints/" + jointName( j ));
            tmsg.addFloatArg( aBody.pos[j].x );
            tmsg.addFloatArg( aBody.pos[j].y );
            tmsg.addFloatArg( aBody.pos[j].z );
            tmsg.addStringArg( aBody.seen[j] ? "Tracked" : "NotTracked" );
            mSender.sendMessage( tmsg, false );
            mNumSent++;
        }
    }

    size_t getNumSent() const { return mNumSent; }
    const string& getHost() const { return mHost; }
    int getPort() const { return mPort; }

    ofParameter< float > minScore, minDepthM, maxDepthM, pitchDeg, rollDeg;
    ofParameter< int > depthWindow, minJoints;
    ofParameter< bool > flipX;

protected:
    glm::mat3 mLevelMatrix{ 1.f };
    bool mUseLevelMatrix = false;
    ofParameterGroup mParams;
    ofxOscSender mSender;
    string mHost = "127.0.0.1";
    int mPort = 9999;
    size_t mNumSent = 0;
};
