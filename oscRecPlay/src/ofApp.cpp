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
    }
    saveSettings();
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
void ofApp::draw() {
    gui.begin();
    ImGui::SetNextWindowPos( ImVec2( 10, 10 ), ImGuiCond_Always );
    ImGui::SetNextWindowSize( ImVec2( ofGetWidth() - 20, ofGetHeight() - 20 ), ImGuiCond_Always );
    ImGui::Begin( "OSC Record Play", nullptr, ImGuiWindowFlags_NoCollapse | ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove );

    ImGui::InputText( "file (in bin/data)", filePath, sizeof( filePath ));
    ImGui::Separator();

    ImGui::TextUnformatted( "Record" );
    ImGui::InputInt( "listen on port", &recPort );
    ImGui::Checkbox( "forward to", &forward );
    ImGui::SameLine();
    ImGui::SetNextItemWidth( 140 );
    ImGui::InputText( "##fwdHost", fwdHost, sizeof( fwdHost ));
    ImGui::SameLine();
    ImGui::SetNextItemWidth( 90 );
    ImGui::InputInt( "##fwdPort", &fwdPort, 0 );
    if( mode == Mode::Recording ) {
        if( ImGui::Button( "Stop recording", ImVec2( 160, 0 ))) stop();
    } else {
        if( ImGui::Button( "Record", ImVec2( 160, 0 ))) startRecording();
    }
    ImGui::Separator();

    ImGui::TextUnformatted( "Play" );
    ImGui::SetNextItemWidth( 140 );
    ImGui::InputText( "host", playHost, sizeof( playHost ));
    ImGui::InputInt( "port", &playPort );
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
