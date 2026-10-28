#pragma once

//--------------------------------------------------------------
//
// OSC Record Play
//
// Records the OSC that femtoSkeletonOsc sends (/bodies/<id>/joints/<Joint> ...) and plays it back
// later, so whatever listens can be developed and shown without the cameras.
//
// Record: listens on a port and writes every packet, with its arrival time, to a file of its own,
// bin/data/takes/take_<date>_<time>.oscrec, so every take is kept. To record while
// the listener is running, point the tracker at this app's port and give a "forward to" address (the
// listener's, such as 127.0.0.1:9999): everything is passed on as it arrives.
//
// Play: sends the recording to a host and port with its original timing, at a chosen speed, once or
// in a loop.
//
// Packets are kept byte for byte (see Recording.h), so nothing is changed on the way. The skeletons in
// them are read as they go by and drawn in 2D: as the sensor sees them, and from above.
// Settings are kept in bin/data/settings.json.
//
//--------------------------------------------------------------

#include "ofMain.h"
#include "ofxImGui.h"
#include "ofxNetwork.h"
#include "Recording.h"
#include "BodyJoints.h"
#include <array>
#include <atomic>
#include <map>
#include <mutex>
#include <set>
#include <thread>

class ofApp : public ofBaseApp {
public:
    void setup() override;
    void update() override;
    void draw() override;
    void exit() override;
    void keyPressed( int key ) override;
    void drawControls();

private:
    enum class Mode { Idle, Recording, Playing };

    void startRecording();
    void startPlaying();
    void stop();
    void recordLoop( std::string path, int port, std::string fwdHost, int fwdPort );
    void playLoop( std::vector<oscrec::Packet> take, std::string host, int port );
    void noteAddresses( const char* data, size_t n );
    void drawFront( const ofRectangle& r );
    void drawTop( const ofRectangle& r );
    void loadSettings();
    void listTakes();
    void saveSettings() const;

    ofxImGui::Gui gui;
    bool showGui = true;     // the controls down the left (h)
    float panelW = 430.f;    // their width, as wide as what is in them (last frame's)

    Mode mode = Mode::Idle;
    std::thread worker;
    std::atomic<bool> stopFlag { false };

    // what the window edits
    char filePath[512] = "take1.oscrec";   // the take to play (in bin/data); a new recording becomes it
    std::vector<std::string> takes;          // every recording, newest first (paths in bin/data)
    std::string shotPath;                    // OSCREC_SHOT: a screenshot to save, then quit
    float takesListedAt = -100.f;
    int recPort = 9998;
    bool forward = true;
    char fwdHost[128] = "127.0.0.1";
    int fwdPort = 9999;
    char playHost[128] = "127.0.0.1";
    int playPort = 9999;
    float speed = 1.f;
    bool loop = true;

    // what the worker reports
    std::atomic<size_t> packets { 0 };
    std::atomic<size_t> bytes { 0 };
    std::atomic<double> seconds { 0.0 };    // elapsed recording time, or the position in the take
    std::atomic<double> length { 0.0 };     // the length of the take being played
    std::atomic<float> speedLive { 1.f };
    std::atomic<bool> workerDone { false };
    std::mutex statusMutex;
    std::string status = "Idle";
    std::string lastAddress;
    std::set<std::string> bodyIds;

    // The latest pose of everyone in the packets going by (sender's frame: metres, x toward the
    // sensor's left, y up, z negative away from it), under statusMutex.
    struct Pose {
        std::array<glm::vec3, bodyjoints::Count> p;
        std::array<bool, bodyjoints::Count> seen{};
        float t = 0.f;   // ofGetElapsedTimef() of its last message
    };
    std::map<std::string, Pose> poses;
};
