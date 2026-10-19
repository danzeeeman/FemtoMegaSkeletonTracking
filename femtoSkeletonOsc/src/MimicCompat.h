//
//  MimicCompat.h
//  Mimic
//
//  Included ahead of every file in the project (see ForcedIncludeFiles in the project).
//  Mimic's code, and the older addons it uses, were written when openFrameworks put the std namespace
//  in scope for you; this OF does not.
//

#pragma once

#ifdef __cplusplus
// with the std namespace in scope, std::byte clashes with the byte typedef in the Windows headers
#define _HAS_STD_BYTE 0
#include <algorithm>
#include <iostream>
#include <list>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
using namespace std;

#include <atomic>
#include <cstdint>
// ofGetFrameNum() belongs to the window, so only the main thread should call it. The main thread keeps this
// up to date, for the log messages in code that runs on the robot thread.
inline std::atomic< uint64_t >& mimicFrameCounter() { static std::atomic< uint64_t > tcount{ 0 }; return tcount; }
#endif
