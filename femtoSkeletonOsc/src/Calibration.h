//
//  Calibration.h
//
//  Where one Femto is, relative to the reference Femto, found with a person as the target: while someone
//  walks about where both cameras can see them, every joint both cameras place is a point seen twice, once
//  in each camera's frame. A rigid transform that carries one set onto the other (Kabsch, inside a RANSAC so a
//  wild joint cannot spoil it) is the camera's pose.
//
//  Joints are only as good as the pose estimate, a few centimetres, so the person has to move through a good
//  part of the shared space (not stand still, not walk a straight line) for the answer to be well pinned down.
//

#pragma once
#include "ofMain.h"
#include <opencv2/core.hpp>
#include <random>

class Calibration {
public:
    struct Result {
        bool ok = false;
        glm::mat4 toRef{ 1.f };   // a point in this camera's frame -> the reference frame
        float rms = 0.f;          // metres, over the inliers
        int inliers = 0;
        int total = 0;
        std::string message;
    };

    // One point, seen by both: `own` in this camera's frame, `ref` in the reference frame.
    void add( const glm::vec3& aOwn, const glm::vec3& aRef ) {
        if( mOwn.size() >= kMaxSamples ) {
            // keep the newest: drop the oldest tenth
            size_t tdrop = kMaxSamples / 10;
            mOwn.erase( mOwn.begin(), mOwn.begin() + tdrop );
            mRef.erase( mRef.begin(), mRef.begin() + tdrop );
        }
        mOwn.push_back( aOwn );
        mRef.push_back( aRef );
    }
    void clear() { mOwn.clear(); mRef.clear(); }
    size_t size() const { return mOwn.size(); }

    // The spread of the points in the reference frame: standard deviation along the least-spread direction.
    // Small means they lie on a plane or a line and the pose is not pinned down. With `aYawOnly` (both cameras
    // levelled, y vertical) only the spread across the floor matters.
    float spread( bool aYawOnly = false ) const {
        if( mRef.size() < 10 ) return 0.f;
        const int td = aYawOnly ? 2 : 3;
        auto comp = [&]( const glm::vec3& p, int i ) { return aYawOnly ? ( i == 0 ? p.x : p.z ) : p[i]; };
        double tc[3] = { 0, 0, 0 };
        for( auto& p : mRef ) for( int i = 0; i < td; i++ ) tc[i] += comp( p, i );
        for( int i = 0; i < td; i++ ) tc[i] /= (double)mRef.size();
        cv::Mat_< double > tH = cv::Mat_< double >::zeros( td, td );
        for( auto& p : mRef ) {
            double d[3];
            for( int i = 0; i < td; i++ ) d[i] = comp( p, i ) - tc[i];
            for( int i = 0; i < td; i++ ) for( int j = 0; j < td; j++ ) tH( i, j ) += d[i] * d[j];
        }
        cv::Mat tw;
        cv::eigen( tH, tw );
        return (float)std::sqrt( std::max( 0.0, tw.at< double >( td - 1 ) / (double)mRef.size() ));
    }

    // `aYawOnly`: both cameras are levelled (from their accelerometers), so the pose is a turn about the vertical
    // and a shift: four numbers instead of six, and much better conditioned.
    Result solve( bool aYawOnly = false, float aInlierM = 0.06f, int aIterations = 400, float aMinSpread = 0.2f ) const {
        Result r;
        r.total = (int)mOwn.size();
        if( mOwn.size() < 60 ) { r.message = "need more points (move around where both cameras see you)"; return r; }
        if( spread( aYawOnly ) < aMinSpread ) { r.message = aYawOnly ? "walk about more: the points do not cover enough of the floor" : "the points are too flat: walk through more of the shared space"; return r; }

        std::mt19937 trng( 12345 );
        std::vector< int > tbestIn;
        const int tn = (int)mOwn.size();
        for( int it = 0; it < aIterations; it++ ) {
            std::vector< int > tidx;
            for( int k = 0; k < 6; k++ ) tidx.push_back( (int)( trng() % tn ));
            glm::mat4 tm;
            if( !fit( tidx, aYawOnly, tm )) continue;
            std::vector< int > tin;
            for( int i = 0; i < tn; i++ )
                if( glm::distance( glm::vec3( tm * glm::vec4( mOwn[i], 1.f )), mRef[i] ) < aInlierM ) tin.push_back( i );
            if( tin.size() > tbestIn.size() ) tbestIn = tin;
        }
        if( tbestIn.size() < 40 ) { r.message = "no consistent pose found: the two cameras are not seeing the same person well"; return r; }

        // refit on the inliers, twice, so the inlier set follows the better pose
        glm::mat4 tm;
        for( int pass = 0; pass < 2; pass++ ) {
            if( !fit( tbestIn, aYawOnly, tm )) { r.message = "fit failed"; return r; }
            std::vector< int > tin;
            for( int i = 0; i < tn; i++ )
                if( glm::distance( glm::vec3( tm * glm::vec4( mOwn[i], 1.f )), mRef[i] ) < aInlierM ) tin.push_back( i );
            if( tin.size() < 40 ) break;
            tbestIn = tin;
        }
        double tsq = 0.0;
        for( int i : tbestIn ) {
            float d = glm::distance( glm::vec3( tm * glm::vec4( mOwn[i], 1.f )), mRef[i] );
            tsq += d * d;
        }
        r.ok = true;
        r.toRef = tm;
        r.inliers = (int)tbestIn.size();
        r.rms = (float)std::sqrt( tsq / tbestIn.size() );
        r.message = "ok";
        return r;
    }

private:
    static constexpr size_t kMaxSamples = 12000;

    // Rigid transform carrying the `own` points at `aIdx` onto the `ref` ones (Kabsch).
    bool fit( const std::vector< int >& aIdx, bool aYawOnly, glm::mat4& aOut ) const {
        if( aIdx.size() < 3 ) return false;
        glm::dvec3 ta( 0.0 ), tb( 0.0 );
        for( int i : aIdx ) { ta += glm::dvec3( mOwn[i] ); tb += glm::dvec3( mRef[i] ); }
        ta /= (double)aIdx.size();
        tb /= (double)aIdx.size();
        if( aYawOnly ) {
            // a turn about y: x' = x cos + z sin, z' = -x sin + z cos
            double ts1 = 0.0, ts2 = 0.0;
            for( int i : aIdx ) {
                glm::dvec3 a = glm::dvec3( mOwn[i] ) - ta, b = glm::dvec3( mRef[i] ) - tb;
                ts1 += a.x * b.x + a.z * b.z;
                ts2 += a.z * b.x - a.x * b.z;
            }
            const double tth = std::atan2( ts2, ts1 ), tc = std::cos( tth ), ts = std::sin( tth );
            glm::dmat3 tg( 1.0 );
            tg[0] = glm::dvec3( tc, 0.0, -ts );
            tg[2] = glm::dvec3( ts, 0.0, tc );
            glm::dvec3 tt = tb - tg * ta;
            glm::mat4 tm( 1.f );
            for( int c = 0; c < 3; c++ ) tm[c] = glm::vec4( glm::vec3( tg[c] ), 0.f );
            tm[3] = glm::vec4( glm::vec3( tt ), 1.f );
            aOut = tm;
            return true;
        }
        cv::Mat_< double > tH = cv::Mat_< double >::zeros( 3, 3 );
        for( int i : aIdx ) {
            glm::dvec3 a = glm::dvec3( mOwn[i] ) - ta, b = glm::dvec3( mRef[i] ) - tb;
            for( int r = 0; r < 3; r++ ) for( int c = 0; c < 3; c++ ) tH( r, c ) += a[r] * b[c];
        }
        cv::SVD tsvd( tH );
        cv::Mat_< double > tR = tsvd.vt.t() * tsvd.u.t();
        if( cv::determinant( tR ) < 0 ) {
            cv::Mat_< double > tV = tsvd.vt.t();
            tV.col( 2 ) *= -1.0;
            tR = tV * tsvd.u.t();
        }
        glm::dmat3 tg;   // column-major
        for( int r = 0; r < 3; r++ ) for( int c = 0; c < 3; c++ ) tg[c][r] = tR( r, c );
        glm::dvec3 tt = tb - tg * ta;
        glm::mat4 tm( 1.f );
        for( int c = 0; c < 3; c++ ) tm[c] = glm::vec4( glm::vec3( tg[c] ), 0.f );
        tm[3] = glm::vec4( glm::vec3( tt ), 1.f );
        aOut = tm;
        return true;
    }

    std::vector< glm::vec3 > mOwn, mRef;
};
