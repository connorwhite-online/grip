# Vision: the wrist camera

GRIP's first job is unloading a printer. It could do that blind, picking from the part positions in the slicer file. But a blind pick fails on a part that shifted, a plate with several parts, a failed print or spaghetti, or a plate that was never cleared. So the arm carries a small USB camera on the side of its hand. The camera finds the parts, picks a grasp for each one, and checks the plate is empty before the next job starts.

> **Status:** the hardware, mount and wiring are designed. The vision software described below is the next step on the roadmap, and nothing here has been tested on a real arm yet.

![Gripper with the camera cradle on its -y side](drawings/gripper.svg)

## Hardware

| Part | Why this one | Cost |
| --- | --- | ---: |
| Arducam UB0234 (OV2710, 1080p, USB 2.0 UVC, 32 x 32 mm board, M12 lens, 105° horizontal) | Plug-and-play on any OS (UVC, no driver). The board is small and has a standard 28 mm hole pattern. The M12 lens can be refocused and swapped. The OV2710 sensor does well in low light. | $35 |
| Slim USB 2.0 cable, 2 m, 3 mm OD or less | The camera's own 1 m lead won't reach the base through the arm. This cable is spliced onto it. | $6 |
| 5 V white LED strip (2835, 8 mm wide), two 2-LED pieces | Even light from the camera's own point of view, so the image looks the same day or night. Runs off the camera's 5 V. | $6 |
| AprilTag tag36h11 (id 0, optionally id 1) and a ChArUco board | Printed on matte paper. The tag tells the camera exactly where the plate is; the board is for calibrating the camera. | $0 |

All of these are in the [BOM](../bom/BOM.md) under *Vision*.

Any computer that already runs the arm can run the vision: a laptop, a mini PC or a Raspberry Pi 5. The pipeline is a few OpenCV calls on one image per pick, not real-time video.

### Where the camera sits

The cradle (`gripper_camera_mount`) bolts to the outside of the rod frame's -y rail, with two M3x12 screws into heat-set inserts. The camera looks along the tool axis, tilted 15° in toward it. That keeps the jaws out of its view, while the fingertips and the space between them are near the middle of the image. The lens sits 76 mm above the J6 flange, beside the jaw tops. The cradle sticks out about 42 mm past the jaws on that side.

The URDF has a fixed frame for the camera, `wrist_camera_optical`, on the board's front face with +Z along the optical axis. That is the CAD position. Hand-eye calibration (below) measures the real one.

## Build

Do this after step 5 of the [gripper build](BUILD.md#5-build-the-gripper), before the gripper goes on the arm.

1. **Inserts.** Set two M3 heat-set inserts in the holes in the outer face of the rod frame's -y rail.
2. **Focus the lens.** Out of the box the lens is focused for 1 m to infinity. GRIP works at 100–300 mm. Plug the camera into a computer, point it at printed text 200 mm away, and turn the lens out (counterclockwise) until the text is sharpest. Lock it with its lock nut, or with a drop of glue on the thread.
3. **Cradle onto the frame.** Put the two M3x12 screws in through the cradle's open back, and drive them through the counterbored holes in its inner wall into the inserts.
4. **Board into the cradle.** Push the lens holder through the front window, with the board's cable connector toward the open back. Fix the board to the four standoffs with M2x5 self-tapping screws.
5. **Lights.** Cut two 2-LED pieces from the 5 V strip. Stick them on the cradle's front face, one on each side of the lens window. Solder their + and − leads to the camera's 5 V and GND at the splice (next section). Four LEDs draw about 150 mA. With the camera's 200 mA, that is well inside a USB 2.0 port's 500 mA.

### The camera cable

The camera's lead is a JST-ZH 4-pin connector on a 1 m USB cable. It is spliced to the slim 2 m cable, so that only the slim cable runs through the arm:

1. Cut the camera's lead about 150 mm from the board connector.
2. Cut the device-end plug off the slim cable. Feed its bare end from the base up through the arm's cable ports alongside the CAN cable, leaving the same 80 mm service loop at each joint.
3. At the gripper, strip both cables and solder them wire to wire: 5 V, GND, D+, D−. Colours vary on cheap cables (usually red, black, green, white), so check each wire with a multimeter against the USB-A pins first. Keep the untwisted length of the D+/D− pair under 10 mm, sleeve each joint in heat shrink, then sleeve the whole splice.
4. From the wrist link to the cradle, the cable runs outside, beside the servo cable, in the same loop around J6.

The wrist's cable ports are 8 mm across. From J4 up, they carry the CAN pair, the power pair, the servo's 3-core cable and the camera cable. Use 22 AWG silicone wire for the power pair from J4 up, and a camera cable no thicker than 3 mm, or the last ports get tight.

At the base, plug the USB-A end into the controlling computer, next to the CAN adapter. On Linux, `v4l2-ctl --list-devices` should list it.

## Calibration

Do these once, and again if you take the camera off.

1. **Lens (intrinsics).** Print a ChArUco board (OpenCV can generate one, for example 7 x 5 squares of 30 mm). Take 25–30 pictures of it at different angles and distances, filling the corners of the frame. Run `cv2.calibrateCamera` on them. Aim for a reprojection error under 0.5 px. If it's worse at the edges, use the rational distortion model (`cv2.CALIB_RATIONAL_MODEL`): the 105° lens bends straight lines noticeably near the corners.
2. **Hand-eye.** Tape the ChArUco board flat on the table in reach. Move the arm through 15–20 poses that look at the board from different directions, with real rotation about all three axes, not just shifts. At each pose, record the tool pose from forward kinematics (the URDF and the joint angles) and the board pose seen by the camera. Then solve with `cv2.calibrateHandEye` (eye-in-hand; Park or Tsai). The result should be within a few millimetres and a degree or two of the CAD frame `wrist_camera_optical`. If it's far off, the camera is mounted wrong or a joint's zero is wrong.
3. **Printer.** Stick tag 0 (about 20 mm) on a flat spot at the front of the bed that moves with the plate and that the camera can see from above. On the A1 mini, the plate covers almost all of the heatbed, so check your printer for a spot; if there isn't one, print a small clip for the front of the bed carriage. The bed slides toward the arm, so the tag gives the plate's position on every cycle. Tag 1 is optional: stuck to the table beside the printer, it tells you if someone has moved the printer. With the plate empty, take one picture from the survey pose (below), and store where the 180 mm print area is relative to tag 0. Also store that empty-plate image as the reference.

## Unloading a plate

This is the loop the software will run. Every step uses standard OpenCV and AprilTag calls.

1. **Wait for the print to finish.** Read the printer's state over its local network interface (the Bambu A1 mini reports job state and bed temperature over LAN MQTT, the same way the Home Assistant integration reads it). Depending on the firmware, this may need LAN-only or developer mode. Wait until the job has finished and the bed is below about 30 °C, when PLA and PETG come off textured PEI easily.
2. **Survey.** Move to the survey pose, with the camera about 210 mm above the plate. J6 turns the camera to the side and J5 tips it level, so it looks within about 6° of straight down. The jaws open wide, to keep the fingers out of the picture. Take a picture and find tag 0. That gives the plate's exact position this cycle, even if the bed parked somewhere slightly different. The [viewer](https://claude.ai/artifact/23hGc6k4nbLyYigruTJS9T) shows this view.
3. **Find the parts.** Warp the image to a flat, top-down view of the print area at 0.25 mm per pixel. Compare it with the empty-plate reference. Anything that differs over more than about 25 mm² is a part: threshold the difference, clean it up with a morphological open, and take the outlines. The textured PEI is dark and even, so most colours stand out at once. For dark filament on a dark plate, use the part positions from the job's 3MF as a guide and look for the edges there.
4. **Choose a grasp.** For each part, fit the smallest rotated rectangle around it. The jaws close across the short side, which must be under 75 mm, since they open to 80. J6 turns to that angle. Parts taller than about 65 mm reach the palm, so read their height from the job's 3MF and grasp them higher up.
5. **Pick.** Lower the fingertips to 3 mm above the plate, close the jaws, and check the servo's position. If the jaws closed well past the part's width, the pick missed: lift, twist J6 by 10° and try once more, then give up and ask for help.
6. **Place** the part in the bin beside the printer.
7. **Check the plate is clear.** Return to the survey pose and compare with the reference again. Only when the plate matches it is the next job allowed to start. This is also how failed prints get caught: a blob that isn't where the job said a part would be means a person should look.

### Software

- **Plain Python:** OpenCV (4.7 or later, which includes the ArUco and ChArUco functions), `pupil-apriltags`, NumPy, and `python-can` for the arm.
- **ROS 2:** `apriltag_ros`, and `easy_handeye2` for the hand-eye step. They use the URDF's `wrist_camera_optical` frame.
