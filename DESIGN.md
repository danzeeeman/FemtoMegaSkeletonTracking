# Design system

One look for the Echo & Narcissus tools: femtoSkeletonOsc and oscRecPlay (this repository) and echoNarcissus (EchoAndNarcissus). Each is a dark, dense, quiet operator console for a live installation, where **a colour always means the same thing**.

It lives in one header, `OperatorTheme.h`. The same file is in `femtoSkeletonOsc/src`, `oscRecPlay/src` and `echoNarcissus/src`; change all three together. Each app calls `optheme::applyImGui()` once after `gui.setup()` and takes every colour from `optheme::`, not from numbers written in place.

echoNarcissus's 3D scene (the white arms, black frame, navy floor and pool) is the artwork's own look and goes into its renders. It is not part of this. The system covers its control window and its debug overlays.

## Principles

- **Quiet surfaces, loud meaning.** Slate greys for everything structural, colour only where it says something.
- **One meaning per colour**, the same in every app. A warning in one is a warning in all.
- **Dense, not cramped.** Tight ImGui spacing (6x4 frame padding, 8x5 item spacing), 4 px rounding, 1 px window borders.
- **Readable.** Every text colour is at least 4.5:1 against what it sits on.
- **Still.** No animation in the controls. What moves is the data: people, arms, gazes.

## Surfaces

| token | hex | for |
|---|---|---|
| `bg` | `#0F172A` | behind everything; 3D and 2D view backgrounds; dark text on action buttons |
| `panel` | `#1B2336` | control windows |
| `raised` | `#272F42` | fields, headers, tabs |
| `raisedHi` | `#334155` | hovered; plain buttons |
| `border` | `#475569` | window, view and panel borders |
| `text` | `#F8FAFC` | text (15.9:1 on panel) |
| `textMuted` | `#94A3B8` | labels, view titles, hints (6.3:1 on panel) |
| `grid` / `gridFine` | white at 28 / 12 alpha | floor grid: metres / half metres |

## Status

| token | hex | means | used for |
|---|---|---|---|
| `ok` | `#22C55E` | live, go | Start moving |
| `active` | `#3B82F6` | the interactive accent | sliders, selection, focus |
| `activeFill` | `#1D4ED8` | the same, under text | a toggle that is on (white text, 6.7:1) |
| `warning` | `#F59E0B` | in between, partial | connecting, no camera yet, not calibrated, no accelerometer, a person seen only as a box |
| `danger` | `#F87171` | stopping, recording, faults | REC, the safety gate (as text) |
| `dangerFill` | `#A84848` | the same, as a button: a soft brick red | Stop recording, Stop this camera, Hold still (white text, 5.4:1) |
| `info` | `#38BDF8` | playing back | Stop playing, "playing …" |

Primary actions are full-width buttons filled with their status colour: `optheme::actionButton(label, colour)`. Its text is white or dark, whichever contrasts more with the fill (`textOn()`). Red buttons use `dangerFill`: dark text on the brighter red only just reached 4.5:1 and was hard to read in the small font.

On / off settings are **toggle buttons**, not checkboxes: `optheme::toggle(label, value)` or `optheme::toggle(ofParameter<bool>&)`. On is filled with `activeFill` and white text, off is plain with muted text. The label ends in "on" or "off", so the state does not rest on colour alone. A setting whose on is a hazard passes `dangerFill` as the on colour: echoNarcissus's "Real arm armed" shows red while armed.

A press always answers: saving shows a green "saved" for 3 s. A step that is hard to undo asks first: "Forget calibration" turns into a red "Really forget?" for 3 s, and only a second press forgets. Status lines carry their meaning in colour as well as words: OSC quiet or not listening, EGM waiting and IK out of reach are `warning`, EGM connected and OSC flowing are `ok`, simulation is muted. An action's two states (Record / Stop recording, Play / Stop playing, Start moving / Hold still) are the same size and in the same place, so the layout does not jump when it changes.

## What is drawn

| token | hex | |
|---|---|---|
| `person` | `#4ADE80` | someone tracked (echoNarcissus). femtoSkeletonOsc and oscRecPlay give each person their own hue, from their id, to tell several apart |
| `fakePerson` | `#A78BFA` | the stand-in person (testing) |
| `boxPerson` | = `warning` | someone found without a skeleton: partial |
| `sideLeft` / `sideRight` | `#60A5FA` / `#F87171` | a body's left and right side, mixed half into its colour (`sided()`), in every app |
| `narcissus` | `#38BDF8` | Narcissus and his gaze: the water |
| `echo` | `#F472B6` | Echo, her gaze, and the person she is with |
| `camera(0..3)` | `#22D3EE` `#A78BFA` `#FACC15` `#FB923C` | each camera: its frustum, its own skeletons, its label. The sensor in echoNarcissus and oscRecPlay is camera 0's |
| `axisX/Y/Z` | red / green / blue | axes, as everywhere |

## Type

ImGui's built-in font in the controls, and openFrameworks' bitmap font for labels over the views (view titles in `textMuted` on `bg`). A sans such as Inter would read better at small sizes; it means bundling the font file and building ImGui's font atlas with it. That is not done yet.

## Review (October 2026)

What the apps did before this, and what changed:

1. **Three backgrounds** (femtoSkeletonOsc `24` with `(14,15,20)` views, oscRecPlay `24` with `(16,17,20)`, echoNarcissus `(12,13,16)`), each with ImGui's stock blue theme. Now one `bg` and `panel` everywhere, and the ImGui style set from tokens.
2. **Status colours chosen case by case**: two ambers, an orange-red, two reds and a cyan, with no fixed meaning. Now `warning`, `danger` and `info`, one meaning each. "No camera at …" and "not calibrated" are warnings: they resolve themselves or need an action, they are not faults.
3. **Orange meant three things**: a box-only person, the person Echo is with, and Echo's gaze; femtoSkeletonOsc's camera 1 was orange too. Now box-only is `warning` amber, Echo and her person are rose, and camera 1 is violet.
4. **Cameras** were green/orange/blue/pink in femtoSkeletonOsc, white or pale yellow elsewhere. Now `camera(i)` everywhere.
5. **Left/right tints** were written out separately in each app. Now one `sided()`.
6. **Primary actions** (Record, Start moving / Hold still) used hand-picked greens and reds, different sizes per state. Now `actionButton` in the status colours, full width.
7. **oscRecPlay's Play address** was two stacked fields while "forward to" was one row. Now both are one row, the fields lined up.

8. **echoNarcissus's checkboxes** are toggles now, its status lines are coloured, and its Engage min / max sliders push each other so they cannot cross.
9. **Reds softened** (October 2026): `danger` `#EF4444` → `#F87171` (text was 4.2:1 on panel, now 5.7:1), `dangerFill` `#B91C1C` → `#A84848`, a brick red that is less loud and still 5.4:1 under white.

Still open: the font (above); text drawn over the 3D views is the bitmap font at one size.
