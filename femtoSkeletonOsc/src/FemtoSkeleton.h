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
            mParams.add( minLegScore.set( "Min leg keypoint score", 0.5f, 0.f, 1.f ));
            mParams.add( reachM.set( "Limb reach from the torso (m)", 0.8f, 0.3f, 2.f ));
        }
        return mParams;
    }

    //--------------------------------------------------------------
    // The 3D point under a colour pixel, in the camera frame (metres). False with no good depth there.
    // The depth picture is as big as the colour one it is registered to, but not always what the
    // intrinsics were read from: unprojectAt() scales to it.
    bool unproject( const ofVec2f& aPix, const ofShortPixels& aDepth, const ofxOrbbecIntrinsics& aK, ofVec3f& aOut ) const {
        int tw = aDepth.getWidth(), th = aDepth.getHeight();
        if( tw < 1 || th < 1 || aK.fx <= 0.f ) return false;
        int tu = (int)roundf( aPix.x ), tv = (int)roundf( aPix.y );
        if( tu < 0 || tv < 0 || tu >= tw || tv >= th ) return false;

        // the near end of the valid depths in a window (a fifth of the way in), so a hole does not spoil a joint
        // and, where the window takes in a limb and what is behind it, the limb wins without one stray near
        // pixel deciding it
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
        std::nth_element( tvals, tvals + tn / 5, tvals + tn );
        return unprojectAt( aPix, tvals[tn / 5], aDepth, aK, aOut );
    }

    // The 3D point under a colour pixel at a given depth (metres), in the camera frame.
    bool unprojectAt( const ofVec2f& aPix, float aZ, const ofShortPixels& aDepth, const ofxOrbbecIntrinsics& aK, ofVec3f& aOut ) const {
        const int tw = aDepth.getWidth(), th = aDepth.getHeight();
        if( tw < 1 || th < 1 || aK.fx <= 0.f || aZ <= 0.f ) return false;
        const float tsx = aK.width > 0 ? (float)tw / aK.width : 1.f;
        const float tsy = aK.height > 0 ? (float)th / aK.height : 1.f;
        aOut.x = ( aPix.x - aK.cx * tsx ) * aZ / ( aK.fx * tsx );
        aOut.y = ( aPix.y - aK.cy * tsy ) * aZ / ( aK.fy * tsy );
        aOut.z = aZ;
        return true;
    }

    //--------------------------------------------------------------
    // The camera hangs tilted, so its y is not up. Pitch (about the camera's x) and roll (about its z), in degrees,
    // level the points; adjust them until the floor and the people stand straight in the plan and in Mimic.
    // With `aLevel` (the camera's accelerometer, see below), that instead of the sliders.
    ofVec3f level( const ofVec3f& aCam, const glm::mat3* aLevel = nullptr ) const {
        if( aLevel ) {
            glm::vec3 tl = *aLevel * glm::vec3( aCam.x, aCam.y, aCam.z );
            return ofVec3f( tl.x, tl.y, tl.z );
        }
        float tp = ofDegToRad( pitchDeg ), tr = ofDegToRad( rollDeg );
        float tcp = cosf( tp ), tsp = sinf( tp ), tcr = cosf( tr ), tsr = sinf( tr );
        ofVec3f t( aCam.x, aCam.y * tcp - aCam.z * tsp, aCam.y * tsp + aCam.z * tcp );
        return ofVec3f( t.x * tcr - t.y * tsr, t.x * tsr + t.y * tcr, t.z );
    }


    //--------------------------------------------------------------
    // False when too few joints could be placed in 3D to be worth sending. `aLevel`: the camera's pitch and roll from
    // its accelerometer instead of the two sliders, a matrix that turns a point in the camera's frame into the same
    // point in a frame with y straight down and z the way the camera faces, flattened onto the horizon. Several
    // cameras' threads build at once, so it is passed in rather than kept here.
    bool build( ofxRtmTrackedPose& aPose, const ofShortPixels& aDepth, const ofxOrbbecIntrinsics& aK, Body& aBody,
                const glm::mat3* aLevel = nullptr ) const {
        aBody = Body();
        aBody.id = aPose.ID;

        const int tnum = (int)aPose.keypoints.size();
        const bool twhole = tnum >= 133;
        // where each joint was found in the picture, to place it again at another depth
        std::array< ofVec2f, TOTAL_JOINTS > tpix;
        std::array< bool, TOTAL_JOINTS > thasPix;
        thasPix.fill( false );

        // a keypoint's place in the picture, or not
        auto pixel = [&]( int aIndex, float aMinScore, ofVec2f& aOut ) -> bool {
            if( aIndex < 0 || aIndex >= tnum ) return false;
            if( aPose.scores.size() <= (size_t)aIndex || aPose.scores[aIndex] < aMinScore ) return false;
            const glm::vec2& tk = aPose.keypoints[aIndex];
            if( !ofxRtmPose::sIsKeypointValid( tk )) return false;
            aOut.set( tk.x, tk.y );
            return true;
        };
        auto set = [&]( int aJoint, const ofVec3f& aCam ) {
            aBody.cam[aJoint] = aCam;
            aBody.seen[aJoint] = true;
        };
        // a joint straight from a keypoint, in the camera's frame (levelled at the end)
        auto direct = [&]( int aJoint, int aKeypoint, bool aLeg = false ) {
            ofVec2f tpx;
            if( !pixel( aKeypoint, aLeg ? std::max( minScore.get(), minLegScore.get() ) : minScore.get(), tpx )) return;
            // a leg at the bottom edge of the picture is mostly out of it, and its keypoint a guess
            if( aLeg && tpx.y > aDepth.getHeight() * 0.97f ) return;
            ofVec3f tp;
            if( !unproject( tpx, aDepth, aK, tp )) return;
            set( aJoint, tp );
            tpix[aJoint] = tpx;
            thasPix[aJoint] = true;
        };

        // COCO-17 indices: 0 nose, 3/4 ears, 5/6 shoulders, 7/8 elbows, 9/10 wrists, 11/12 hips, 13/14 knees, 15/16 ankles
        direct( SHOULDER_LEFT, 5 );  direct( SHOULDER_RIGHT, 6 );
        direct( ELBOW_LEFT, 7 );     direct( ELBOW_RIGHT, 8 );
        direct( WRIST_LEFT, 9 );     direct( WRIST_RIGHT, 10 );
        direct( HIP_LEFT, 11 );      direct( HIP_RIGHT, 12 );
        direct( KNEE_LEFT, 13, true );  direct( KNEE_RIGHT, 14, true );
        direct( ANKLE_LEFT, 15, true ); direct( ANKLE_RIGHT, 16, true );
        if( twhole ) {
            // the big toes
            direct( FOOT_LEFT, 17, true );
            direct( FOOT_RIGHT, 20, true );
            // hands: left 91..111 (wrist, thumb 1-4, index 1-4, middle 1-4...), right 112..132. The hand is the
            // middle finger's knuckle, the tip its end.
            for( int tside = 0; tside < 2; tside++ ) {
                const int tbase = tside == 0 ? 91 : 112;
                direct( tside == 0 ? HAND_LEFT : HAND_RIGHT, tbase + 9 );
                direct( tside == 0 ? HAND_TIP_LEFT : HAND_TIP_RIGHT, tbase + 12 );
                direct( tside == 0 ? THUMB_LEFT : THUMB_RIGHT, tbase + 4 );
            }
        }

        // The joints that came out impossible. A thin part (a hand, a finger, an arm side on) can miss the depth
        // under it and take the wall behind: its keypoint is right but its depth is not. The torso's depth is
        // sound, so a limb joint too far from its parent, or too far in front of or behind the torso, is put at its
        // parent's depth instead, and dropped if even that does not fit.
        float ttorso[4];
        int tnt = 0;
        for( int j : { SHOULDER_LEFT, SHOULDER_RIGHT, HIP_LEFT, HIP_RIGHT } ) if( aBody.seen[j] ) ttorso[tnt++] = aBody.cam[j].z;
        std::sort( ttorso, ttorso + tnt );
        const bool thasTorso = tnt > 0;
        const float ttorsoZ = thasTorso ? ttorso[tnt / 2] : 0.f;
        auto tooFar = [&]( const ofVec3f& p ) { return thasTorso && std::abs( p.z - ttorsoZ ) > reachM; };
        auto fit = [&]( int aParent, int aChild, float aMaxLen ) {
            if( !aBody.seen[aChild] ) return;
            if( !aBody.seen[aParent] ) {
                if( tooFar( aBody.cam[aChild] )) aBody.seen[aChild] = false;
                return;
            }
            const ofVec3f& tpar = aBody.cam[aParent];
            if( !tooFar( aBody.cam[aChild] ) && aBody.cam[aChild].distance( tpar ) <= aMaxLen * 1.3f ) return;
            ofVec3f tagain;
            if( thasPix[aChild] && unprojectAt( tpix[aChild], tpar.z, aDepth, aK, tagain ) && tagain.distance( tpar ) <= aMaxLen * 1.5f ) aBody.cam[aChild] = tagain;
            else aBody.seen[aChild] = false;
        };
        // from the torso outwards, so each joint is checked against a parent that has been
        fit( SHOULDER_LEFT, ELBOW_LEFT, 0.45f );    fit( SHOULDER_RIGHT, ELBOW_RIGHT, 0.45f );
        fit( ELBOW_LEFT, WRIST_LEFT, 0.40f );       fit( ELBOW_RIGHT, WRIST_RIGHT, 0.40f );
        fit( WRIST_LEFT, HAND_LEFT, 0.15f );        fit( WRIST_RIGHT, HAND_RIGHT, 0.15f );
        fit( HAND_LEFT, HAND_TIP_LEFT, 0.15f );     fit( HAND_RIGHT, HAND_TIP_RIGHT, 0.15f );
        fit( WRIST_LEFT, THUMB_LEFT, 0.15f );       fit( WRIST_RIGHT, THUMB_RIGHT, 0.15f );
        fit( HIP_LEFT, KNEE_LEFT, 0.60f );          fit( HIP_RIGHT, KNEE_RIGHT, 0.60f );
        fit( KNEE_LEFT, ANKLE_LEFT, 0.60f );        fit( KNEE_RIGHT, ANKLE_RIGHT, 0.60f );
        fit( ANKLE_LEFT, FOOT_LEFT, 0.30f );        fit( ANKLE_RIGHT, FOOT_RIGHT, 0.30f );

        // no toe: the foot is at the ankle. No hand: the hand is at the wrist, so the arm still ends somewhere.
        if( !aBody.seen[FOOT_LEFT] && aBody.seen[ANKLE_LEFT] ) set( FOOT_LEFT, aBody.cam[ANKLE_LEFT] );
        if( !aBody.seen[FOOT_RIGHT] && aBody.seen[ANKLE_RIGHT] ) set( FOOT_RIGHT, aBody.cam[ANKLE_RIGHT] );
        if( !aBody.seen[HAND_LEFT] && aBody.seen[WRIST_LEFT] ) set( HAND_LEFT, aBody.cam[WRIST_LEFT] );
        if( !aBody.seen[HAND_RIGHT] && aBody.seen[WRIST_RIGHT] ) set( HAND_RIGHT, aBody.cam[WRIST_RIGHT] );

        // the spine and head are made from what was found
        const bool tsh = aBody.seen[SHOULDER_LEFT] && aBody.seen[SHOULDER_RIGHT];
        const bool thp = aBody.seen[HIP_LEFT] && aBody.seen[HIP_RIGHT];
        if( tsh ) set( SPINE_SHOULDER, ( aBody.cam[SHOULDER_LEFT] + aBody.cam[SHOULDER_RIGHT] ) * 0.5f );
        if( thp ) set( SPINE_BASE, ( aBody.cam[HIP_LEFT] + aBody.cam[HIP_RIGHT] ) * 0.5f );
        if( tsh && thp ) set( SPINE_MID, ( aBody.cam[SPINE_SHOULDER] + aBody.cam[SPINE_BASE] ) * 0.5f );

        // the head: between the ears, else the nose. Hair at its edge can take the wall's depth too, so a head
        // too far from the shoulders is put at their depth.
        ofVec2f tl, tr, thpx;
        bool theadPix = false;
        if( pixel( 3, minScore, tl ) && pixel( 4, minScore, tr )) { thpx = ( tl + tr ) * 0.5f; theadPix = true; }
        else if( pixel( 0, minScore, thpx )) theadPix = true;
        ofVec3f thead;
        if( theadPix && unproject( thpx, aDepth, aK, thead )) {
            if( tsh && ( tooFar( thead ) || thead.distance( aBody.cam[SPINE_SHOULDER] ) > 0.5f )) {
                if( !unprojectAt( thpx, aBody.cam[SPINE_SHOULDER].z, aDepth, aK, thead )) theadPix = false;
            }
            if( theadPix ) set( HEAD, thead );
        }
        // the neck is a little way up from the shoulders to the head
        if( tsh && aBody.seen[HEAD] ) set( NECK, aBody.cam[SPINE_SHOULDER] + ( aBody.cam[HEAD] - aBody.cam[SPINE_SHOULDER] ) * 0.3f );

        // level everything, then to the Kinect's frame
        for( int j = 0; j < TOTAL_JOINTS; j++ ) {
            if( !aBody.seen[j] ) continue;
            aBody.cam[j] = level( aBody.cam[j], aLevel );
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
    // A person the detector found but whose skeleton could not be built: their box, in 3D. `centre` and `size`
    // in the camera's levelled frame (x right, y down, z forward, metres), axis-aligned there.
    struct Box {
        uint64_t id = 0;
        glm::vec3 centre{ 0.f }, size{ 0.f };
    };

    // The detector's box out to the person's depth: the near part of the depths in the middle of the box (the
    // body, not what is behind it at the box's edges), the box's corners projected out to it, and a body's
    // thickness behind. False when there is too little depth in it.
    bool buildBox( const ofxRtmTrackedPose& aPose, const ofShortPixels& aDepth, const ofxOrbbecIntrinsics& aK, Box& aBox,
                   const glm::mat3* aLevel = nullptr ) const {
        ofRectangle tr = aPose.box.outRect;
        if( tr.width < 2.f || tr.height < 2.f ) tr = aPose.getBoundingBox();
        const int tw = aDepth.getWidth(), th = aDepth.getHeight();
        if( tw < 1 || th < 1 || tr.width < 2.f || tr.height < 2.f ) return false;
        // the middle half of the box, sampled on a grid of about 40 x 40
        const int x0 = std::max( 0, (int)( tr.x + tr.width * 0.25f )), x1 = std::min( tw - 1, (int)( tr.x + tr.width * 0.75f ));
        const int y0 = std::max( 0, (int)( tr.y + tr.height * 0.25f )), y1 = std::min( th - 1, (int)( tr.y + tr.height * 0.75f ));
        const int tsx = std::max( 1, ( x1 - x0 ) / 40 ), tsy = std::max( 1, ( y1 - y0 ) / 40 );
        std::vector< float > tvals;
        tvals.reserve( 1700 );
        const unsigned short* tdata = aDepth.getData();
        for( int y = y0; y <= y1; y += tsy ) {
            for( int x = x0; x <= x1; x += tsx ) {
                const unsigned short td = tdata[y * tw + x];
                if( td == 0 ) continue;
                const float tm = td * 0.001f;
                if( tm >= minDepthM && tm <= maxDepthM ) tvals.push_back( tm );
            }
        }
        if( tvals.size() < 20 ) return false;
        const size_t tk = tvals.size() * 3 / 10;
        std::nth_element( tvals.begin(), tvals.begin() + tk, tvals.end());
        const float tz = tvals[tk];
        ofVec3f ta, tb;
        if( !unprojectAt( ofVec2f( tr.x, tr.y ), tz, aDepth, aK, ta ) || !unprojectAt( ofVec2f( tr.getRight(), tr.getBottom() ), tz, aDepth, aK, tb )) return false;
        const float tthick = 0.3f;
        const ofVec3f tc = level( ofVec3f(( ta.x + tb.x ) * 0.5f, ( ta.y + tb.y ) * 0.5f, tz + tthick * 0.5f ), aLevel );
        aBox.id = aPose.ID;
        aBox.centre = glm::vec3( tc.x, tc.y, tc.z );
        aBox.size = glm::vec3( std::abs( tb.x - ta.x ), std::abs( tb.y - ta.y ), tthick );
        return true;
    }

    // /boxes/<id> cx cy cz sx sy sz, in the sensor's frame as the joints are sent (x per flipX, y up, z negative
    // away from it)
    void sendBox( uint64_t aId, const glm::vec3& aCentre, const glm::vec3& aSize ) {
        ofxOscMessage tmsg;
        tmsg.setAddress( "/boxes/" + ofToString( aId ));
        tmsg.addFloatArg( flipX ? -aCentre.x : aCentre.x );
        tmsg.addFloatArg( -aCentre.y );
        tmsg.addFloatArg( -aCentre.z );
        tmsg.addFloatArg( aSize.x );
        tmsg.addFloatArg( aSize.y );
        tmsg.addFloatArg( aSize.z );
        mSender.sendMessage( tmsg, false );
        mNumSent++;
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

    ofParameter< float > minScore, minDepthM, maxDepthM, pitchDeg, rollDeg, minLegScore, reachM;
    ofParameter< int > depthWindow, minJoints;
    ofParameter< bool > flipX;

protected:
    ofParameterGroup mParams;
    ofxOscSender mSender;
    string mHost = "127.0.0.1";
    int mPort = 9999;
    size_t mNumSent = 0;
};
