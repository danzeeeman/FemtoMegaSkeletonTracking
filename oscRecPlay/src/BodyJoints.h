#pragma once
#include <array>
#include <string>
#include <utility>

// The 25 joints femtoSkeletonOsc sends as /bodies/<id>/joints/<Joint> (Kinect v2 names, in the order of
// FemtoSkeleton::Joint), and the bones between them, for drawing a whole person.
namespace bodyjoints {

enum Joint {
    SpineBase = 0, SpineMid, SpineShoulder, Neck, Head,
    ShoulderLeft, ElbowLeft, WristLeft, HandLeft, HandTipLeft, ThumbLeft,
    ShoulderRight, ElbowRight, WristRight, HandRight, HandTipRight, ThumbRight,
    HipLeft, KneeLeft, AnkleLeft, FootLeft,
    HipRight, KneeRight, AnkleRight, FootRight,
    Count
};

inline const char* name(int j) {
    static const char* const names[Count] = {
        "SpineBase", "SpineMid", "SpineShoulder", "Neck", "Head",
        "ShoulderLeft", "ElbowLeft", "WristLeft", "HandLeft", "HandTipLeft", "ThumbLeft",
        "ShoulderRight", "ElbowRight", "WristRight", "HandRight", "HandTipRight", "ThumbRight",
        "HipLeft", "KneeLeft", "AnkleLeft", "FootLeft",
        "HipRight", "KneeRight", "AnkleRight", "FootRight" };
    return j >= 0 && j < Count ? names[j] : "";
}

// -1 for a name that is not one of the 25.
inline int fromName(const std::string& n) {
    for (int j = 0; j < Count; ++j) if (n == name(j)) return j;
    return -1;
}

// Which side a joint is on: -1 left, 1 right, 0 the middle (spine, neck, head).
inline int side(int j) {
    if (j >= ShoulderLeft && j <= ThumbLeft) return -1;
    if (j >= ShoulderRight && j <= ThumbRight) return 1;
    if (j >= HipLeft && j <= FootLeft) return -1;
    if (j >= HipRight && j <= FootRight) return 1;
    return 0;
}

inline const std::array<std::pair<int, int>, 24>& bones() {
    static const std::array<std::pair<int, int>, 24> b = {{
        {SpineBase, SpineMid}, {SpineMid, SpineShoulder}, {SpineShoulder, Neck}, {Neck, Head},
        {SpineShoulder, ShoulderLeft}, {ShoulderLeft, ElbowLeft}, {ElbowLeft, WristLeft}, {WristLeft, HandLeft},
        {HandLeft, HandTipLeft}, {WristLeft, ThumbLeft},
        {SpineShoulder, ShoulderRight}, {ShoulderRight, ElbowRight}, {ElbowRight, WristRight}, {WristRight, HandRight},
        {HandRight, HandTipRight}, {WristRight, ThumbRight},
        {SpineBase, HipLeft}, {HipLeft, KneeLeft}, {KneeLeft, AnkleLeft}, {AnkleLeft, FootLeft},
        {SpineBase, HipRight}, {HipRight, KneeRight}, {KneeRight, AnkleRight}, {AnkleRight, FootRight},
    }};
    return b;
}

}
