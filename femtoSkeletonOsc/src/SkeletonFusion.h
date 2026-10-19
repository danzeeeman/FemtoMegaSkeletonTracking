//
//  SkeletonFusion.h
//
//  Several cameras see the same person. Each camera's bodies, put in the reference camera's frame, are
//  grouped: bodies from different cameras that are close together are one person. Each group becomes (or
//  updates) a track with an id of its own, so a person keeps the same id whichever cameras can see them, and
//  every joint is the average of the cameras that saw it.
//

#pragma once
#include "ofMain.h"
#include "FemtoSkeleton.h"
#include <algorithm>

class SkeletonFusion {
public:
    struct Detection {
        int camera = 0;
        FemtoSkeleton::Body body;   // `cam` joints in the reference frame
    };
    struct Track {
        uint64_t id = 0;
        FemtoSkeleton::Body body;   // merged, `cam` joints in the reference frame
        glm::vec3 centre{ 0.f };
        double lastSeen = 0.0;
        int cameras = 0;            // how many cameras saw this person in the last update
        bool updated = false;       // seen in the last update()
    };

    ofParameterGroup params;
    ofParameter< float > mergeDist{ "Same person within (m)", 0.5f, 0.1f, 1.5f };
    ofParameter< float > trackDist{ "Track follows within (m)", 0.8f, 0.2f, 2.f };
    ofParameter< float > lostSec{ "Forget a person after (s)", 1.f, 0.2f, 5.f };

    SkeletonFusion() {
        params.setName( "Fusion" );
        params.add( mergeDist );
        params.add( trackDist );
        params.add( lostSec );
    }

    const std::vector< Track >& tracks() const { return mTracks; }

    // The middle of a body: the torso joints if there are any, else everything seen.
    static bool centreOf( const FemtoSkeleton::Body& aBody, glm::vec3& aOut ) {
        using F = FemtoSkeleton;
        static const int ttorso[] = { F::SPINE_MID, F::SPINE_BASE, F::SPINE_SHOULDER, F::SHOULDER_LEFT, F::SHOULDER_RIGHT, F::HIP_LEFT, F::HIP_RIGHT };
        glm::vec3 tsum( 0.f );
        int tn = 0;
        for( int j : ttorso ) {
            if( !aBody.seen[j] ) continue;
            tsum += glm::vec3( aBody.cam[j].x, aBody.cam[j].y, aBody.cam[j].z );
            tn++;
        }
        if( tn == 0 ) {
            for( int j = 0; j < F::TOTAL_JOINTS; j++ ) {
                if( !aBody.seen[j] ) continue;
                tsum += glm::vec3( aBody.cam[j].x, aBody.cam[j].y, aBody.cam[j].z );
                tn++;
            }
        }
        if( tn == 0 ) return false;
        aOut = tsum / (float)tn;
        return true;
    }

    // Folds in what the cameras see now. Returns the tracks, with `updated` set on those that were seen.
    const std::vector< Track >& update( const std::vector< Detection >& aDets, double aNow ) {
        struct Cluster {
            std::vector< const Detection* > members;
            glm::vec3 centre{ 0.f };
        };
        std::vector< Cluster > tclusters;
        for( auto& d : aDets ) {
            glm::vec3 tc;
            if( !centreOf( d.body, tc )) continue;
            Cluster* tbest = nullptr;
            float tbestD = mergeDist;
            for( auto& cl : tclusters ) {
                bool thasCamera = false;
                for( auto* m : cl.members ) if( m->camera == d.camera ) thasCamera = true;
                if( thasCamera ) continue;
                float td = glm::distance( cl.centre, tc );
                if( td < tbestD ) { tbestD = td; tbest = &cl; }
            }
            if( !tbest ) {
                tclusters.emplace_back();
                tbest = &tclusters.back();
            }
            tbest->members.push_back( &d );
            glm::vec3 tsum( 0.f );
            for( auto* m : tbest->members ) { glm::vec3 c; centreOf( m->body, c ); tsum += c; }
            tbest->centre = tsum / (float)tbest->members.size();
        }

        for( auto& t : mTracks ) t.updated = false;

        // clusters to tracks, nearest first
        struct Pair { float d; int cluster, track; };
        std::vector< Pair > tpairs;
        for( int i = 0; i < (int)tclusters.size(); i++ )
            for( int j = 0; j < (int)mTracks.size(); j++ ) {
                float td = glm::distance( tclusters[i].centre, mTracks[j].centre );
                if( td < trackDist ) tpairs.push_back( { td, i, j } );
            }
        std::sort( tpairs.begin(), tpairs.end(), []( const Pair& a, const Pair& b ) { return a.d < b.d; } );
        std::vector< int > tclusterTrack( tclusters.size(), -1 );
        std::vector< bool > ttrackUsed( mTracks.size(), false );
        for( auto& p : tpairs ) {
            if( tclusterTrack[p.cluster] >= 0 || ttrackUsed[p.track] ) continue;
            tclusterTrack[p.cluster] = p.track;
            ttrackUsed[p.track] = true;
        }

        for( int i = 0; i < (int)tclusters.size(); i++ ) {
            int tt = tclusterTrack[i];
            if( tt < 0 ) {
                Track tn;
                tn.id = mNextId++;
                mTracks.push_back( tn );
                tt = (int)mTracks.size() - 1;
            }
            Track& t = mTracks[tt];
            merge( tclusters[i].members, t );
            t.centre = tclusters[i].centre;
            t.lastSeen = aNow;
            t.cameras = (int)tclusters[i].members.size();
            t.updated = true;
        }

        mTracks.erase( std::remove_if( mTracks.begin(), mTracks.end(), [&]( const Track& t ) { return aNow - t.lastSeen > lostSec; } ), mTracks.end() );
        return mTracks;
    }

private:
    static void merge( const std::vector< const Detection* >& aMembers, Track& aTrack ) {
        FemtoSkeleton::Body tb;
        tb.id = aTrack.id;
        for( int j = 0; j < FemtoSkeleton::TOTAL_JOINTS; j++ ) {
            glm::vec3 tsum( 0.f );
            int tn = 0;
            for( auto* m : aMembers ) {
                if( !m->body.seen[j] ) continue;
                tsum += glm::vec3( m->body.cam[j].x, m->body.cam[j].y, m->body.cam[j].z );
                tn++;
            }
            if( tn == 0 ) continue;
            tsum /= (float)tn;
            tb.cam[j] = ofVec3f( tsum.x, tsum.y, tsum.z );
            tb.seen[j] = true;
        }
        aTrack.body = tb;
    }

    std::vector< Track > mTracks;
    uint64_t mNextId = 1;
};
