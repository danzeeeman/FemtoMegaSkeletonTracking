# SkeletonTracking

Finding people with Orbbec Femto Mega depth cameras and sending their skeletons out over OSC, and recording and replaying that OSC. Split out of [EchoAndNarcissus](https://github.com/danzeeeman/EchoAndNarcissus), whose `echoNarcissus` app listens to it.

```
Femto Mega cameras --> femtoSkeletonOsc --OSC--> (oscRecPlay) --> echoNarcissus, or anything that reads it
```

Both are openFrameworks apps. Clone this repository into `<openFrameworks>/apps/`, so each app sits three folders below openFrameworks.

## femtoSkeletonOsc

Finds the people in front of one or more networked Femto Megas (Ethernet / PoE) and sends their skeletons:

```
/bodies/<id>/joints/<Joint>   x y z "Tracked" | "NotTracked"      25 Kinect-style joints, metres
/boxes/<id>                   cx cy cz sx sy sz                   a person found but without a skeleton
```

- **People and keypoints:** YOLO11n-pose ([ofxYoloPose](https://github.com/danzeeeman/ofxYoloPose), the default), or YOLOX + RTMW whole-body ([ofxRtmPoseTracker](https://github.com/danzeeeman/ofxRtmPoseTracker)). The models are downloaded into `bin/data/models` on the first run.
- **3D:** each keypoint's depth comes from the Femto's depth image, registered to the colour image on the camera itself. It is levelled with the camera's accelerometer, and limb joints with impossible depths are fixed or dropped.
- **Several cameras:** each runs on a thread of its own. Calibrated against camera 0, their skeletons are merged into one per person.
- **Record / play back:** the cameras' frames (colour, depth, accelerometer) are recorded to `bin/data/femto_takes/*.femto`. A take plays back in place of a camera, through the same tracking. The Record button or `r` records every camera.
- **Views:** the camera images with the poses on them, and the joined space in 3D, in perspective and from above.

## oscRecPlay

Records the OSC that femtoSkeletonOsc sends to `bin/data/takes/*.oscrec`, every take kept, and plays it back with its timing. While recording it can forward everything on (to echoNarcissus, say). The skeletons going by are drawn in 2D, as the sensor sees them and from above.

## Setup

1. `./get_addons.sh` clones the addons into `<openFrameworks>/addons`. On macOS it also fetches the Orbbec SDK into ofxOrbbec.
2. In each app: `make`, then `make run`. On macOS, allow the app on the local network when asked (System Settings → Privacy & Security → Local Network).
