#pragma once

#include <cstdint>
#include <cstring>
#include <fstream>
#include <functional>
#include <string>
#include <vector>

// A recording of raw OSC packets, kept byte for byte with the time each one arrived, so
// messages and bundles play back exactly as they were sent.
//
// File: "OSCREC1\n", then for each packet a little-endian double (seconds since the first
// packet), a uint32 length and the packet bytes.
namespace oscrec {

inline const char* kMagic = "OSCREC1\n";
constexpr size_t kMagicLen = 8;

struct Packet {
    double t = 0.0;
    std::vector<char> data;
};

inline void writeHeader( std::ostream& out ) {
    out.write( kMagic, kMagicLen );
}

inline void writePacket( std::ostream& out, double t, const char* data, uint32_t n ) {
    out.write( reinterpret_cast<const char*>( &t ), sizeof( t ));
    out.write( reinterpret_cast<const char*>( &n ), sizeof( n ));
    out.write( data, n );
}

// Returns false if the file is missing or is not a recording. A cut-off last packet is dropped.
inline bool read( const std::string& path, std::vector<Packet>& packets ) {
    packets.clear();
    std::ifstream in( path, std::ios::binary );
    char magic[kMagicLen];
    if( !in.read( magic, kMagicLen ) || std::memcmp( magic, kMagic, kMagicLen ) != 0 ) return false;
    for( ;; ) {
        Packet p;
        uint32_t n = 0;
        if( !in.read( reinterpret_cast<char*>( &p.t ), sizeof( p.t ))) break;
        if( !in.read( reinterpret_cast<char*>( &n ), sizeof( n ))) break;
        p.data.resize( n );
        if( !in.read( p.data.data(), n )) break;
        packets.push_back( std::move( p ));
    }
    return true;
}

// Calls `fn` with every message in a packet (its bytes and length), looking inside bundles.
inline void forEachMessage( const char* d, size_t n, const std::function<void( const char*, size_t )>& fn ) {
    if( n == 0 ) return;
    if( n >= 16 && std::memcmp( d, "#bundle", 8 ) == 0 ) {
        size_t i = 16;
        while( i + 4 <= n ) {
            const uint8_t* b = reinterpret_cast<const uint8_t*>( d + i );
            size_t len = ( size_t( b[0] ) << 24 ) | ( size_t( b[1] ) << 16 ) | ( size_t( b[2] ) << 8 ) | size_t( b[3] );
            i += 4;
            if( i + len > n ) return;
            forEachMessage( d + i, len, fn );
            i += len;
        }
    } else if( d[0] == '/' ) {
        fn( d, n );
    }
}

// A joint message, /bodies/<id>/joints/<Joint> x y z ["Tracked"|"NotTracked"]: false for anything else.
inline bool parseJoint( const char* d, size_t n, std::string& id, std::string& joint, float xyz[3], bool& tracked ) {
    // OSC strings end with a NUL and are padded to 4 bytes; numbers are big-endian
    auto str = [&]( size_t& i, std::string& out ) {
        size_t e = i;
        while( e < n && d[e] != '\0' ) e++;
        if( e >= n ) return false;
        out.assign( d + i, e - i );
        i = ( e + 4 ) & ~size_t( 3 );
        return true;
    };
    auto f32 = [&]( size_t& i, float& out ) {
        if( i + 4 > n ) return false;
        const uint8_t* b = reinterpret_cast<const uint8_t*>( d + i );
        uint32_t u = ( uint32_t( b[0] ) << 24 ) | ( uint32_t( b[1] ) << 16 ) | ( uint32_t( b[2] ) << 8 ) | uint32_t( b[3] );
        std::memcpy( &out, &u, 4 );
        i += 4;
        return true;
    };
    size_t i = 0;
    std::string address, tags;
    if( !str( i, address ) || !str( i, tags )) return false;
    static const std::string prefix = "/bodies/";
    if( address.compare( 0, prefix.size(), prefix ) != 0 ) return false;
    const size_t slash = address.find( "/joints/", prefix.size() );
    if( slash == std::string::npos ) return false;
    id = address.substr( prefix.size(), slash - prefix.size() );
    joint = address.substr( slash + 8 );
    if( tags.size() < 4 || tags.compare( 0, 4, ",fff" ) != 0 ) return false;
    for( int k = 0; k < 3; k++ ) if( !f32( i, xyz[k] )) return false;
    tracked = true;
    std::string state;
    if( tags.size() > 4 && tags[4] == 's' && str( i, state )) tracked = state == "Tracked";
    return true;
}

// Calls `fn` with the address pattern of every message in a packet, looking inside bundles.
inline void forEachAddress( const char* d, size_t n, const std::function<void( const std::string& )>& fn ) {
    if( n == 0 ) return;
    if( n >= 16 && std::memcmp( d, "#bundle", 8 ) == 0 ) {
        size_t i = 16;
        while( i + 4 <= n ) {
            const uint8_t* b = reinterpret_cast<const uint8_t*>( d + i );
            size_t len = ( size_t( b[0] ) << 24 ) | ( size_t( b[1] ) << 16 ) | ( size_t( b[2] ) << 8 ) | size_t( b[3] );
            i += 4;
            if( i + len > n ) return;
            forEachAddress( d + i, len, fn );
            i += len;
        }
    } else if( d[0] == '/' ) {
        size_t len = 0;
        while( len < n && d[len] != '\0' ) len++;
        fn( std::string( d, len ));
    }
}

// The id in "/bodies/<id>/...", or "" for any other address.
inline std::string bodyId( const std::string& address ) {
    static const std::string prefix = "/bodies/";
    if( address.compare( 0, prefix.size(), prefix ) != 0 ) return "";
    size_t end = address.find( '/', prefix.size() );
    return address.substr( prefix.size(), end == std::string::npos ? std::string::npos : end - prefix.size() );
}

} // namespace oscrec
