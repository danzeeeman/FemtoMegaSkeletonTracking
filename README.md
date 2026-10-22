# SkeletonTracking

Finding people with Orbbec Femto Mega depth cameras and sending their skeletons out over OSC, and recording and replaying that OSC. Split out of [EchoAndNarcissus](https://github.com/danzeeeman/EchoAndNarcissus), whose `echoNarcissus` app listens to it.

```
Femto Mega cameras --> femtoSkeletonOsc --OSC--> (oscRecPlay) --> echoNarcissus, or anything that reads it
```

Both are openFrameworks apps. Clone this repository into `<openFrameworks>/apps/`, so each app sits three folders below openFrameworks.

- [Setup](#setup)
- [femtoSkeletonOsc](#femtoskeletonosc): [the window](#the-window), [what it sends](#what-it-sends), [how a skeleton is made](#how-a-skeleton-is-made), [several cameras](#several-cameras), [recording the cameras](#recording-the-cameras), [settings](#settings), [files](#files), [testing without cameras](#testing-without-cameras)
- [oscRecPlay](#oscrecplay)

## Setup

1. `./get_addons.sh` clones the addons into `<openFrameworks>/addons`: ofxImGui, ofxOnnx, ofxOrbbec, ofxYoloPose and ofxRtmPoseTracker. Pass `--update` to pull ones you already have. On macOS it also fetches the Orbbec SDK (v2.10.6) into ofxOrbbec. ofxOpenCv, ofxOsc and ofxNetwork come with openFrameworks.
2. In each app run `make` (Release; `make Debug` for the debug build), then `make run`.
3. On macOS, allow the app on the local network when asked (System Settings → Privacy & Security → Local Network); it cannot reach the cameras otherwise. The apps ask for it in their `openFrameworks-Info.plist`, and the build signs the whole bundle (ad hoc, with `of.entitlements`) so macOS reads it. A rebuild changes the signature, so macOS may ask again.

## femtoSkeletonOsc

Finds the people in front of one or more networked Femto Megas (Ethernet / PoE) and sends their skeletons out over OSC.

### The window

- **Left**, the controls:
  - the people being sent, OSC message count and frame rate;
  - Output;
  - the **Record** button;
  - a panel per camera;
  - Fusion, skeleton, and the pose model's settings.
- **Bottom right**, each camera's colour picture with the poses found in it. A person found without a skeleton is outlined in amber with "box".
- **Top right**, the joined space in 3D, in camera 0's frame:
  - **perspective**: drag to orbit, wheel to zoom;
  - **from above**: camera 0 at the bottom, 1 m grid.

  Each camera is drawn as a frustum in its colour, with its own skeletons thin in that colour. The merged people are bold and labelled `person N (k cameras)`. Boxes are orange wireframes.

Keys: `space` sending on / off, `r` record every camera / stop.

### What it sends

```
/bodies/<id>/joints/<Joint>   x y z "Tracked" | "NotTracked"
/boxes/<id>                   cx cy cz sx sy sz
```

- **Joints:** 25 per body, named as the Kinect v2 names them (SpineBase, SpineMid, SpineShoulder, Neck, Head, Shoulder/Elbow/Wrist/Hand/HandTip/Thumb Left and Right, Hip/Knee/Ankle/Foot Left and Right). Every joint is sent for every body, `NotTracked` when it was not placed.
- **Axes:** metres in camera 0's levelled frame, the way TrackerFemto and Mimic expect them. y is up, z is negative away from the sensor, and x is toward the sensor's left (with "Flip X" on).
- **Ids:** a body's id is the merged person's and stays with them while they are followed.
- **Boxes:** `/boxes/<id>` is a person the detector found whose skeleton could not be built (arms crossed, turned away, half out of view): their centre and size in metres, same axes. A box's id is `camera × 1000000 + the tracker's id`, so it never collides with a body's.
- **Send to:** the OSC host and port in Output. **Apply OSC address** after changing them.

### How a skeleton is made

Each camera has a thread of its own that does all of this per frame, so a frame's poses always meet that frame's depth.

1. **Frames.**
   - The Femto streams 1280x720 colour and 640x576 depth (narrow field, unbinned) at 30 fps.
   - The depth is registered to the colour **on the camera** (hardware D2C, with its factory calibration), so every colour pixel has a depth.
   - Depth and colour are paired by time (frame sync), and only complete pairs are used.
2. **People and keypoints.** One of two models, chosen with **Pose model YOLO11n-pose (else RTMW)**:
   - **YOLO11n-pose** ([ofxYoloPose](https://github.com/danzeeeman/ofxYoloPose)), the default: finds people and their 17 body keypoints in one pass, about 30 times a second. No fingers or toes, so hands are put at the wrists and feet at the ankles.
   - **YOLOX-tiny + RTMW-x-l** ([ofxRtmPoseTracker](https://github.com/danzeeeman/ofxRtmPoseTracker)): finds people, then puts 133 whole-body keypoints on each, fingers and toes included. Slower.

   Both run through ONNX Runtime (CoreML on macOS). The models are downloaded into `bin/data/models` on the first run, and each person keeps an id from frame to frame.
3. **Depth for each keypoint.**
   - The depths in a small window round it (**Depth window**) are taken, and the **near end** of them is used (a fifth of the way in), not the median.
   - A limb is in front of whatever is behind it, so this keeps a hand from taking the wall's depth.
   - Depths outside **Min / Max depth** are ignored.
4. **Fixing what cannot be.** From the torso out, a limb joint that ends up farther from its parent than the bone could be (a wrist 60 cm from its elbow, say), or more than **Limb reach** in front of or behind the torso, is put at its parent's depth instead. It is dropped if even that does not fit. A head too far from the shoulders gets their depth. Leg keypoints need **Min leg keypoint score** and are dropped at the bottom edge of the picture, where legs out of view are only guessed.
5. **Levelled.**
   - The camera's accelerometer gives which way is up, and the points are turned so y is vertical: the camera may hang tilted.
   - Without the accelerometer, **Camera pitch / roll** do it by hand.
   - The panel shows the measured pitch and roll. Leave **Camera Upside Down** off: the Femto reports up. Tick it only if the 3D view comes out upside down (an accelerometer that reports gravity).
6. **A body or a box.**
   - With at least **Joints to send a body** placed, it is a body; the spine, neck and head are made from what was found.
   - Otherwise the detector's box is taken out to the person's depth (the near part of the depths in its middle) as a 3D box, with a body's thickness behind.
7. **Merged and sent.**
   - A second thread takes every camera's new bodies, puts the calibrated cameras' bodies into camera 0's frame and merges them (see [several cameras](#several-cameras)).
   - It sends them as soon as any camera has something new, not at the window's frame rate.

### Several cameras

Up to four Femto Megas can watch the same space. Camera 0 is the **reference**: the skeletons go out in its frame. Every other camera is placed relative to it by a **calibration** you do by walking about, with no target or markers. Where several cameras see the same person, their skeletons are merged into one person with one id.

**1. Network.**
- Each Femto needs an address of its own. The factory address is `192.168.1.10`; change the others' with Orbbec Viewer, for example to `.11`, `.12` and `.13`.
- The computer needs an address in the same subnet, such as `192.168.1.2`.
- All the cameras use the same port, `Camera port (all)`, 8090 by default.
- PoE switches work.

**2. Placing them.**
- Each camera must **share a good part of its view with camera 0**: calibration needs a person both can see at once.
- The cameras do not need to be level: each one's accelerometer measures its pitch and roll.
- Keep "Level from accelerometer" ticked. With every camera levelled, calibration only has to find a turn about the vertical and a shift, which takes far less walking.

**3. Turning them on.**
- In each `Camera N` panel tick **Enabled** and type its **Address**.
- It connects within a few seconds. The panel shows its serial, frame count and accelerometer reading, and its picture joins the row along the top.
- A camera that is not calibrated yet is drawn and tracked, but its people are **not sent** ("not calibrated: its people are not sent").

**4. Calibrating a camera** (each of cameras 1-3, against camera 0):

1. Open the camera's panel and tick **Collect points**.
2. Have **one person, alone**, walk about where this camera and camera 0 both see them, for 20-30 seconds.
   - Cover the shared floor: walk loops, turns and side steps, not one straight line, and move the arms.
   - A point is added for every joint both cameras place, but only when both see exactly one person within 0.1 s of each other.
   - The panel shows the point count and their spread.
3. Press **Solve**.
   - It needs at least 60 points, spread at least 20 cm across the floor (in 3D for a camera without the accelerometer).
   - It finds the camera's pose robustly (RANSAC: points more than 6 cm off are left out, and it needs 40 that agree), so a few wild joints do not matter.
   - It reports **rms**, how far the two cameras' joints still disagree over the points that fit, and how many points fit.
   - **rms under about 5 cm is good.** Over 10 cm usually means the person was not alone, or the shared view is small. Press **Clear points** and walk again.
   - "need more points": keep walking. "the points do not cover enough of the floor": walk more of the shared space. "no consistent pose found": the two cameras were not seeing the same person well.
4. Solving saves the result to `bin/data/calibration.json` and unticks Collect points. The camera's people are sent from then on.

**5. Checking it.**
- In the 3D views each camera's own skeletons are drawn thin in its colour, and the merged person bold.
- With a good calibration the thin skeletons from two cameras lie on top of each other, and the label says `person N (2 cameras)`.
- Each camera's frustum shows where it is.

**Moving a camera** means calibrating it again: **Forget calibration**, then steps 4.1-4.4. Moving camera 0 means calibrating all the others.

**Merging** (the `Fusion` panel):
- Bodies from different cameras within **Same person within** (0.5 m) of each other are one person, and each joint is the average of the cameras that saw it.
- A person keeps their id while they move less than **Track follows within** (0.8 m) between updates, and is forgotten after **Forget a person after** (1 s) unseen.

The cameras run independently, without the sync cable. For two cameras to agree on someone moving fast they have to see them at nearly the same moment, which at 30 fps is within a frame.

### Recording the cameras

- **What is recorded:** what each camera gives (the colour, the depth registered to it, and the accelerometer's up vector), frame by frame with its time, to `bin/data/femto_takes/cam<N>_<date>_<time>.femto`.
- **Playing a take back:** it plays in place of a camera, through the same pose model, skeletons and OSC. This is how to try settings on the same movement again, or work without the cameras.
- **Record** at the top, or `r`, records every connected camera at once (one file each, the same time in their names); press again to stop. Each camera's panel also has **Record / play back** to record just that camera.
- **Playing:** in **Record / play back**, pick a take and press **Play it as this camera**. The camera is let go and the take feeds it, with a progress bar, **Loop** and **Speed** (0.1-2x). **Back to the camera** reconnects.
- **Size:** about 9.5 MB a second, about 570 MB a minute. The colour is JPEG (quality 92); the depth is kept exactly (zlib). Writing happens on a thread of its own, so recording does not slow the tracking; frames it could not keep up with are counted as "passed over".
- **Code:** the recorder and player are in ofxOrbbec (`ofxOrbbecRecording.h`), with the file format described there.

### Settings

**Output**

| | default | |
|---|---|---|
| OSC host, OSC port | 127.0.0.1, 9999 | where the skeletons go (Apply OSC address) |
| Send skeletons | on | off: tracked and drawn, nothing sent (`space`) |
| Camera port (all) | 8090 | the Femtos' network port |
| Colour width, height | 0, 0 | 0 = the camera's fastest colour mode (1280x720) |
| Floor below camera 0 (m) | 1.5 | where the 3D views draw the floor. **Floor from feet** sets it from the feet of whoever is in view |
| Pose model YOLO11n-pose (else RTMW) | on | the pose model |

**Camera N:** Enabled, Address, Level from accelerometer (on), Camera Upside Down (off: turn on only if the 3D view is upside down), Record / play back, and for cameras 1-3 the calibration.

**skeleton**

| | default | |
|---|---|---|
| Min keypoint score | 0.35 | a keypoint less sure than this is not used |
| Depth window (px) | 5 | the window a keypoint's depth is taken from |
| Min / Max depth (m) | 0.3 / 5 | depths outside are ignored |
| Camera pitch / roll (deg) | 0 / 0 | levelling by hand, for a camera not levelled by its accelerometer |
| Flip X | on | x toward the sensor's left, as the Kinect sends it |
| Joints to send a body | 6 | fewer placed: sent as a box |
| Min leg keypoint score | 0.5 | stricter for legs, often half out of view |
| Limb reach from the torso (m) | 0.8 | a limb joint farther in front of or behind the torso is re-placed |

**Fusion:** Same person within, Track follows within, Forget a person after (see [several cameras](#several-cameras)).

**YOLO pose** (with YOLO11n-pose):
- Min person score 0.4
- Overlap to merge 0.5
- Overlap to follow 0.3
- Forget a person after 0.5 s
- Smooth: a One Euro filter on the keypoints. **Min cutoff** 1.5 Hz: lower is steadier standing still. **Speed response** 0.01: higher is less lag on quick moves.

**Pose tracker** (with RTMW): ofxRtmPoseTracker's detector and tracking thresholds.

Settings are saved to `bin/data/settings.xml` on **Save settings** and on quitting.

### Files

In `bin/data`:

| | |
|---|---|
| `settings.xml` | the settings |
| `calibration.json` | each calibrated camera's pose relative to camera 0, its rms and inlier count |
| `models/` | the pose models, downloaded on the first run |
| `femto_takes/` | recordings of the cameras |
| `femtoSkeletonOsc.log` | the log, when the app is started without a terminal (from Finder) |

Debug builds (`make Debug`, `make RunDebug`) also log once a second where skeletons are lost: camera frames, tracker runs, people found, keypoints with depth, bodies built, ids, joints per body, and the accelerometer.

### Testing without cameras

| | |
|---|---|
| `FEMTO_DEMO=1` | two stand-in cameras, calibrated, watch someone walk a loop and wave |
| `FEMTO_PLAY=femto_takes/<take>.femto` | camera 0 plays that take from the start |
| `FEMTO_RECORD=1` | camera 0 is recorded from when it connects |
| `TRACKER_QUIT_AFTER=<s>` | quit after that many seconds, logging the counts |
| `FEMTO_SHOT=<file.png>` | save a screenshot after 6 s and quit |
| `FEMTO_SELFTEST=1` | check calibration and fusion on made-up data, log the results, quit |

For example `FEMTO_PLAY=femto_takes/cam0_20261022_062955.femto TRACKER_QUIT_AFTER=30 make RunDebug`.

## oscRecPlay

Records the OSC that femtoSkeletonOsc sends and plays it back later with its timing, so whatever listens (echoNarcissus) can be worked on and shown without the cameras.

- **Record:**
  - It listens on a port (9998), and each take goes to a file of its own, `bin/data/takes/take_<date>_<time>.oscrec`.
  - To record while echoNarcissus runs, point femtoSkeletonOsc at 9998 and tick **forward to** `127.0.0.1:9999`: every packet is passed on as it arrives.
  - Packets are kept byte for byte, so messages and bundles play back exactly as sent.
- **Recordings:**
  - Every take is listed, newest first (older ones in `bin/data` too).
  - Click one to play it. A new recording is selected when it stops.
- **Play:** to a host and port (127.0.0.1:9999), at 0.1-4x speed, once or in a loop.
- **The skeletons going by** are drawn in 2D while recording or playing:
  - as the sensor sees them (its left on the left);
  - from above (the sensor at the bottom, its field of view drawn in);
  - each person in a colour of their own, left side blue, right side red, with their id and distance.
- **Ports** are typed in. Settings are kept in `bin/data/settings.json`.
