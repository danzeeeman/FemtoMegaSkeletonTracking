#include "ofApp.h"
#include <chrono>

using Clock = std::chrono::steady_clock;

static double since( Clock::time_point t0 ) {
    return std::chrono::duration<double>( Clock::now() - t0 ).count();
}

//--------------------------------------------------------------
void ofApp::setup() {
    ofSetFrameRate( 60 );
    ofSetBackgroundColor( 24 );
    gui.setup();
    loadSettings();
}

//--------------------------------------------------------------
// Every recording: those in bin/data/takes and any older ones in bin/data itself, newest first.
void ofApp::listTakes() {
    takesListedAt = ofGetElapsedTimef();
    std::vector<std::pair<std::time_t, std::string>> found;
    for( std::string dir : { std::string( "takes" ), std::string( "" ) } ) {
        ofDirectory d( dir.empty() ? ofToDataPath( "", true ) : ofToDataPath( dir, true ));
        if( !d.exists() ) continue;
        d.allowExt( "oscrec" );
        d.listDir();
        for( auto& f : d ) {
            const std::string rel = dir.empty() ? f.getFileName() : dir + "/" + f.getFileName();
            found.push_back( { std::filesystem::last_write_time( f.path() ).time_since_epoch().count(), rel } );
        }
    }
    std::sort( found.begin(), found.end(), []( auto& a, auto& b ) { return a.first > b.first; } );
    takes.clear();
    for( auto& f : found ) takes.push_back( f.second );
}

//--------------------------------------------------------------
void ofApp::loadSettings() {
    if( !ofFile::doesFileExist( "settings.json" )) return;
    ofJson j = ofLoadJson( "settings.json" );
    if( j.is_null() ) return;
    auto str = [&]( const char* key, char* dst, size_t n ) {
        if( j.contains( key ) && j[key].is_string() ) std::snprintf( dst, n, "%s", j[key].get<std::string>().c_str() );
    };
    str( "file", filePath, sizeof( filePath ));
    str( "forwardHost", fwdHost, sizeof( fwdHost ));
    str( "playHost", playHost, sizeof( playHost ));
    recPort = j.value( "recordPort", recPort );
    fwdPort = j.value( "forwardPort", fwdPort );
    playPort = j.value( "playPort", playPort );
    forward = j.value( "forward", forward );
    speed = j.value( "speed", speed );
    loop = j.value( "loop", loop );
}

void ofApp::saveSettings() const {
    ofJson j;
    j["file"] = filePath;
    j["recordPort"] = recPort;
    j["forward"] = forward;
    j["forwardHost"] = fwdHost;
    j["forwardPort"] = fwdPort;
    j["playHost"] = playHost;
    j["playPort"] = playPort;
    j["speed"] = speed;
    j["loop"] = loop;
    ofSavePrettyJson( "settings.json", j );
}

//--------------------------------------------------------------
void ofApp::noteAddresses( const char* data, size_t n ) {
    std::lock_guard<std::mutex> lock( statusMutex );
    oscrec::forEachAddress( data, n, [&]( const std::string& a ) {
        lastAddress = a;
        std::string id = oscrec::bodyId( a );
        if( !id.empty() ) bodyIds.insert( id );
    });
    const float now = ofGetElapsedTimef();
    oscrec::forEachMessage( data, n, [&]( const char* m, size_t len ) {
        std::string id, joint;
        float xyz[3];
        bool tracked = false;
        if( !oscrec::parseJoint( m, len, id, joint, xyz, tracked )) return;
        const int j = bodyjoints::fromName( joint );
        if( j < 0 ) return;
        Pose& p = poses[id];
        p.p[j] = glm::vec3( xyz[0], xyz[1], xyz[2] );
        p.seen[j] = tracked;
        p.t = now;
    });
}

//--------------------------------------------------------------
namespace {
// A person's colour from their id, and the left side tinted blue, the right red.
ofColor colourOf( const std::string& id ) {
    return ofColor::fromHsb( ( std::hash<std::string>{}( id ) * 47 ) % 255, 150, 255 );
}
ofColor tint( const ofColor& c, int joint ) {
    const int s = bodyjoints::side( joint );
    return s < 0 ? c.getLerped( ofColor( 80, 150, 255 ), 0.5f ) : s > 0 ? c.getLerped( ofColor( 255, 90, 80 ), 0.5f ) : c;
}

// A panel's frame, a grid every half metre (every metre brighter), and its title.
void drawPanel( const ofRectangle& r, const std::string& title, const std::function<glm::vec2( float, float )>& toScreen,
                float x0, float x1, float y0, float y1 ) {
    ofSetColor( 16, 17, 20 );
    ofDrawRectangle( r );
    for( float v = std::ceil( x0 * 2.f ) / 2.f; v <= x1; v += 0.5f ) {
        ofSetColor( 255, std::fmod( std::abs( v ), 1.f ) < 0.01f ? 34 : 14 );
        ofDrawLine( toScreen( v, y0 ), toScreen( v, y1 ));
    }
    for( float v = std::ceil( y0 * 2.f ) / 2.f; v <= y1; v += 0.5f ) {
        ofSetColor( 255, std::fmod( std::abs( v ), 1.f ) < 0.01f ? 34 : 14 );
        ofDrawLine( toScreen( x0, v ), toScreen( x1, v ));
    }
    ofNoFill();
    ofSetColor( 70, 74, 82 );
    ofDrawRectangle( r );
    ofFill();
    ofSetColor( 200 );
    ofDrawBitmapString( title, r.x + 8, r.y + 16 );
}
}

// As the sensor sees them: its left on the left (x is negated), y up, the sensor at the cross.
void ofApp::drawFront( const ofRectangle& r ) {
    const float x0 = -2.5f, x1 = 2.5f, y0 = -1.4f, y1 = 1.4f;
    const float s = std::min( r.width / ( x1 - x0 ), r.height / ( y1 - y0 ));
    const glm::vec2 c = r.getCenter();
    auto toScreen = [&]( float x, float y ) { return glm::vec2( c.x - x * s, c.y - y * s ); };
    drawPanel( r, "as the sensor sees it (grid 0.5 m, cross: the sensor)", [&]( float a, float b ) { return toScreen( -a, b ); }, x0, x1, y0, y1 );
    ofSetColor( 255, 230, 120 );
    ofDrawLine( c - glm::vec2( 8, 0 ), c + glm::vec2( 8, 0 ));
    ofDrawLine( c - glm::vec2( 0, 8 ), c + glm::vec2( 0, 8 ));

    std::lock_guard<std::mutex> lock( statusMutex );
    const float now = ofGetElapsedTimef();
    for( auto& [id, p] : poses ) {
        if( now - p.t > 0.5f ) continue;
        const ofColor col = colourOf( id );
        ofSetLineWidth( 3 );
        for( const auto& [a, b] : bodyjoints::bones() ) {
            if( !p.seen[a] || !p.seen[b] ) continue;
            ofSetColor( tint( col, b ));
            ofDrawLine( toScreen( p.p[a].x, p.p[a].y ), toScreen( p.p[b].x, p.p[b].y ));
        }
        ofSetLineWidth( 1 );
        for( int j = 0; j < bodyjoints::Count; j++ ) {
            if( !p.seen[j] ) continue;
            ofSetColor( tint( col, j ));
            ofDrawCircle( toScreen( p.p[j].x, p.p[j].y ), j == bodyjoints::Head ? 0.1f * s : 3.f );
        }
        const int top = p.seen[bodyjoints::Head] ? bodyjoints::Head : bodyjoints::SpineShoulder;
        ofSetColor( col );
        const glm::vec2 lp = toScreen( p.p[top].x, p.p[top].y ) + glm::vec2( 0.12f * s, -0.12f * s );
        ofDrawBitmapString( id + "  " + ofToString( -p.p[bodyjoints::SpineMid].z, 2 ) + " m", lp );
    }
}

// From above: the sensor at the bottom looking up the panel, its field of view drawn in.
void ofApp::drawTop( const ofRectangle& r ) {
    const float x0 = -3.f, x1 = 3.f, d0 = -0.3f, d1 = 5.5f;
    const float s = std::min( r.width / ( x1 - x0 ), r.height / ( d1 - d0 ));
    const glm::vec2 o( r.getCenter().x, r.getBottom() + d0 * s );   // the sensor
    auto toScreen = [&]( float x, float d ) { return glm::vec2( o.x - x * s, o.y - d * s ); };
    drawPanel( r, "from above (sensor at the bottom, looking up)", [&]( float a, float b ) { return toScreen( -a, b ); }, x0, x1, d0, d1 );
    // the colour camera's field of view, 81 degrees across
    const float half = std::tan( glm::radians( 40.5f )) * d1;
    ofSetColor( 255, 230, 120, 90 );
    ofDrawLine( o, toScreen( half, d1 ));
    ofDrawLine( o, toScreen( -half, d1 ));
    ofSetColor( 255, 230, 120 );
    ofDrawCircle( o, 4 );

    std::lock_guard<std::mutex> lock( statusMutex );
    const float now = ofGetElapsedTimef();
    for( auto& [id, p] : poses ) {
        if( now - p.t > 0.5f ) continue;
        const ofColor col = colourOf( id );
        ofSetLineWidth( 3 );
        for( const auto& [a, b] : bodyjoints::bones() ) {
            if( !p.seen[a] || !p.seen[b] ) continue;
            ofSetColor( tint( col, b ));
            ofDrawLine( toScreen( p.p[a].x, -p.p[a].z ), toScreen( p.p[b].x, -p.p[b].z ));
        }
        ofSetLineWidth( 1 );
        for( int j = 0; j < bodyjoints::Count; j++ ) {
            if( !p.seen[j] ) continue;
            ofSetColor( tint( col, j ));
            ofDrawCircle( toScreen( p.p[j].x, -p.p[j].z ), j == bodyjoints::Head ? 0.1f * s : 3.f );
        }
    }
}

//--------------------------------------------------------------
void ofApp::recordLoop( std::string path, int port, std::string fwdHost, int fwdPort ) {
    auto setStatus = [&]( const std::string& s ) {
        std::lock_guard<std::mutex> lock( statusMutex );
        status = s;
    };

    ofxUDPManager rx;
    rx.Create();
    rx.SetReuseAddress( true );
    rx.SetNonBlocking( true );
    if( !rx.Bind( port )) {
        setStatus( "Could not listen on port " + std::to_string( port ) + " (in use?)" );
        workerDone = true;
        return;
    }

    ofxUDPManager tx;
    bool forwarding = !fwdHost.empty() && fwdPort > 0;
    if( forwarding ) {
        tx.Create();
        tx.SetNonBlocking( true );
        forwarding = tx.Connect( fwdHost.c_str(), fwdPort );
    }

    std::ofstream out( path, std::ios::binary );
    if( !out ) {
        setStatus( "Could not write " + path );
        workerDone = true;
        return;
    }
    oscrec::writeHeader( out );
    setStatus( "Recording, waiting for the first packet" );

    std::vector<char> buf( 65536 );
    bool started = false;
    Clock::time_point t0;
    while( !stopFlag ) {
        int n = rx.Receive( buf.data(), (int)buf.size() );
        if( n <= 0 ) {
            std::this_thread::sleep_for( std::chrono::milliseconds( 1 ));
            continue;
        }
        if( !started ) {
            started = true;
            t0 = Clock::now();
            setStatus( "Recording" );
        }
        double t = since( t0 );
        oscrec::writePacket( out, t, buf.data(), (uint32_t)n );
        if( forwarding ) tx.Send( buf.data(), n );
        noteAddresses( buf.data(), n );
        packets++;
        bytes += n;
        seconds = t;
    }
    out.close();
    rx.Close();
    if( forwarding ) tx.Close();
    setStatus( "Saved " + std::to_string( packets.load() ) + " packets to " + path );
    workerDone = true;
}

//--------------------------------------------------------------
void ofApp::playLoop( std::vector<oscrec::Packet> take, std::string host, int port ) {
    ofxUDPManager tx;
    tx.Create();
    tx.SetNonBlocking( true );
    if( !tx.Connect( host.c_str(), port )) {
        std::lock_guard<std::mutex> lock( statusMutex );
        status = "Could not connect to " + host + ":" + std::to_string( port );
        workerDone = true;
        return;
    }
    {
        std::lock_guard<std::mutex> lock( statusMutex );
        status = "Playing";
    }

    // The take is followed by its own clock; changing the speed mid-take re-anchors it so the
    // position does not jump.
    do {
        double pos = 0.0;                       // take time reached at the anchor
        auto anchor = Clock::now();
        float anchorSpeed = speedLive;
        for( auto& p : take ) {
            for( ;; ) {
                if( stopFlag ) break;
                float sp = speedLive;
                if( sp != anchorSpeed ) {
                    pos += since( anchor ) * anchorSpeed;
                    anchor = Clock::now();
                    anchorSpeed = sp;
                }
                double now = pos + since( anchor ) * anchorSpeed;
                double wait = ( p.t - now ) / std::max( anchorSpeed, 0.01f );
                if( wait <= 0 ) break;
                std::this_thread::sleep_for( std::chrono::duration<double>( std::min( wait, 0.005 )));
            }
            if( stopFlag ) break;
            tx.Send( p.data.data(), (int)p.data.size() );
            noteAddresses( p.data.data(), p.data.size() );
            packets++;
            bytes += p.data.size();
            seconds = p.t;
        }
    } while( loop && !stopFlag );

    tx.Close();
    {
        std::lock_guard<std::mutex> lock( statusMutex );
        status = stopFlag ? "Stopped" : "Finished";
    }
    workerDone = true;
}

//--------------------------------------------------------------
void ofApp::startRecording() {
    stop();
    packets = 0;
    bytes = 0;
    seconds = 0.0;
    {
        std::lock_guard<std::mutex> lock( statusMutex );
        lastAddress.clear();
        bodyIds.clear();
        poses.clear();
    }
    saveSettings();
    // a file of its own for every take, so none is lost to the next one
    ofDirectory::createDirectory( ofToDataPath( "takes", true ), false, true );
    std::snprintf( filePath, sizeof( filePath ), "%s", ( "takes/take_" + ofGetTimestampString( "%Y%m%d_%H%M%S" ) + ".oscrec" ).c_str() );
    takesListedAt = -100.f;
    std::string path = ofToDataPath( filePath, true );
    stopFlag = false;
    workerDone = false;
    mode = Mode::Recording;
    worker = std::thread( &ofApp::recordLoop, this, path, recPort, forward ? std::string( fwdHost ) : std::string(), fwdPort );
}

void ofApp::startPlaying() {
    stop();
    std::vector<oscrec::Packet> take;
    if( !oscrec::read( ofToDataPath( filePath, true ), take )) {
        std::lock_guard<std::mutex> lock( statusMutex );
        status = std::string( "Not a recording: " ) + filePath;
        return;
    }
    if( take.empty() ) {
        std::lock_guard<std::mutex> lock( statusMutex );
        status = "The recording is empty";
        return;
    }
    packets = 0;
    bytes = 0;
    seconds = 0.0;
    length = take.back().t;
    speedLive = speed;
    {
        std::lock_guard<std::mutex> lock( statusMutex );
        lastAddress.clear();
        bodyIds.clear();
        poses.clear();
    }
    saveSettings();
    stopFlag = false;
    workerDone = false;
    mode = Mode::Playing;
    worker = std::thread( &ofApp::playLoop, this, std::move( take ), std::string( playHost ), playPort );
}

void ofApp::stop() {
    stopFlag = true;
    if( worker.joinable() ) worker.join();
    mode = Mode::Idle;
}

//--------------------------------------------------------------
void ofApp::update() {
    if( mode != Mode::Idle && workerDone ) {
        if( worker.joinable() ) worker.join();
        mode = Mode::Idle;
    }
    if( mode == Mode::Playing ) speedLive = speed;
}

//--------------------------------------------------------------
// A port: typed in, numbers only, no +/- buttons, kept to 1..65535.
static void portInput( const char* aLabel, int& aPort ) {
    if( ImGui::InputInt( aLabel, &aPort, 0, 0 )) aPort = ofClamp( aPort, 1, 65535 );
}

//--------------------------------------------------------------
void ofApp::draw() {
    // the skeletons going by, right of the controls: as the sensor sees them above, from above below
    const float panelW = 430.f, gap = 10.f;
    const float vx = panelW + 2.f * gap, vw = ofGetWidth() - vx - gap, vh = ( ofGetHeight() - 3.f * gap ) * 0.5f;
    drawFront( ofRectangle( vx, gap, vw, vh ));
    drawTop( ofRectangle( vx, 2.f * gap + vh, vw, vh ));

    gui.begin();
    ImGui::SetNextWindowPos( ImVec2( gap, gap ), ImGuiCond_Always );
    ImGui::SetNextWindowSize( ImVec2( panelW, ofGetHeight() - 2.f * gap ), ImGuiCond_Always );
    ImGui::Begin( "OSC Record Play", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove );

    // the recordings: click one to play it
    if( ofGetElapsedTimef() - takesListedAt > 2.f ) listTakes();
    ImGui::Text( "Recordings (%zu), bin/data/takes", takes.size() );
    if( ImGui::ListBoxHeader( "##takes", ImVec2( -1, 170 ))) {
        for( auto& t : takes ) {
            const bool tsel = t == filePath;
            if( ImGui::Selectable( t.c_str(), tsel ) && mode != Mode::Recording ) std::snprintf( filePath, sizeof( filePath ), "%s", t.c_str() );
            if( tsel && ImGui::IsWindowAppearing() ) ImGui::SetScrollHereY();
        }
        ImGui::ListBoxFooter();
    }
    ImGui::Text( "selected: %s", filePath );
    ImGui::Separator();

    ImGui::TextUnformatted( "Record" );
    portInput( "listen on port", recPort );
    ImGui::Checkbox( "forward to", &forward );
    ImGui::SameLine();
    ImGui::SetNextItemWidth( 140 );
    ImGui::InputText( "##fwdHost", fwdHost, sizeof( fwdHost ));
    ImGui::SameLine();
    ImGui::SetNextItemWidth( 90 );
    portInput( "##fwdPort", fwdPort );
    if( mode == Mode::Recording ) {
        if( ImGui::Button( "Stop recording", ImVec2( 160, 0 ))) stop();
    } else {
        if( ImGui::Button( "Record", ImVec2( 160, 0 ))) startRecording();
    }
    ImGui::Separator();

    ImGui::TextUnformatted( "Play" );
    ImGui::SetNextItemWidth( 140 );
    ImGui::InputText( "host", playHost, sizeof( playHost ));
    portInput( "port", playPort );
    ImGui::SliderFloat( "speed", &speed, 0.1f, 4.f, "%.2fx" );
    ImGui::Checkbox( "loop", &loop );
    if( mode == Mode::Playing ) {
        if( ImGui::Button( "Stop playing", ImVec2( 160, 0 ))) stop();
    } else {
        if( ImGui::Button( "Play", ImVec2( 160, 0 ))) startPlaying();
    }
    ImGui::Separator();

    std::string st, last;
    size_t people;
    {
        std::lock_guard<std::mutex> lock( statusMutex );
        st = status;
        last = lastAddress;
        people = bodyIds.size();
    }
    ImGui::Text( "%s", st.c_str() );
    ImGui::Text( "packets %zu   bytes %zu   people seen %zu", packets.load(), bytes.load(), people );
    if( mode == Mode::Playing ) {
        float frac = length > 0 ? (float)( seconds / length ) : 0.f;
        std::string label = ofToString( seconds.load(), 1 ) + " / " + ofToString( length.load(), 1 ) + " s";
        ImGui::ProgressBar( frac, ImVec2( -1, 0 ), label.c_str() );
    } else if( mode == Mode::Recording ) {
        ImGui::Text( "%.1f s", seconds.load() );
    }
    ImGui::TextWrapped( "last: %s", last.c_str() );

    ImGui::End();
    gui.end();
}

//--------------------------------------------------------------
void ofApp::exit() {
    stop();
    saveSettings();
}
