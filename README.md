# Openpose PTZ control

Uses [openpose](https://github.com/CMU-Perceptual-Computing-Lab/openpose) to identify people in an video feed (currently only via webcamera) and issue VISCA over IP PTZ commands to a networked camera. MQTT is used to issue commands to turn the control on/off.

Currently tested and used with a BirdDog P200 using `ffmpeg` with NDI support to feed a v4l2 loopback device on Ubuntu. [Companion](https://github.com/bitfocus/companion) on a StreamDeck, and its' MQTT module, is used to toggle automatic control on/off. It's setup and run in Docker for tidiness sake.

It only tracks the first person it finds, and only moves left/right.

This is the third iteration of this program, after trying various face/people tracking options.

## Setup

* [Install Docker and nvidia-container-toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#installing-on-ubuntu-and-debian), and confirm it works.
* Clone or download this repository:
```
$ wget -O openpose-ptz-control.tar.gz https://github.com/Cameron-D/openpose-ptz-control/archive/main.tar.gz
$ tar xvf openpose-ptz-control.tar.gz
```
* Build the docker container
```
$ cd openpose-ptz-control
$ docker build -t ptzcontrol:11.1 .
```
* Wait a while for it to download and build... ☕

## Running

Running it assumes the webcam/video feed is available at `/dev/video0`...

If you have ffmpeg with NDI support, the following will work to expose it as a video device (limiting to 10 FPS to reduce processing later):
```
sudo modprobe v4l2loopback
ffmpeg -f libndi_newtek -extra_ips "10.1.1.174" -i "BIRDDOG-ABC123 (CAM)" -vf "fps=fps=10" -pix_fmt yuv420p -f v4l2 /dev/video0
```

At this point *an MQTT broker is required*, even if you aren't using any form of toggle to turn the automatic control on/off.

```
docker run -d --restart unless-stopped --name mosquitto -p 1883:1883 eclipse-mosquitto 
```

**Launch the PTZ control container:**
* Set `VISCA_IP` to the camera's IP/hostname
* Set `MQTT_HOST` to the IP/hostname of the MQTT broker (probably the address of the control computer, if you used the above command to run mosquitto)
* Set `CONTROL=False` causes it to automatically start controlling the camera. Remove this line if you would like control to default to OFF (and to turn on/toggle later via a StreamDeck).  

```
$ docker run --gpus all --name ptztrack --restart unless-stopped -it \
    -e VISCA_IP=10.1.1.174 \
    -e MQTT_HOST=10.1.1.175 \
    -e CONTROL=False \
    --device /dev/video0 ptztrack:11.1
```

If all goes well it should start up with no errors, print out in the console when it's moving, and actually move the camera.

## WIshlist of things I might add one day

* ✅ Smooth acceleration for panning (start slow, accelerate if the person nears the edges of the frame)
* Support for multiple people in view, but following only one
* Read direct NDI frames rather than relying on a custom build of ffmpeg and v4l2loopback
* Track a persons head rather than the full body bounding box
* An alternative library for people detection (i.e. tf-pose-estimation seems to be faster?)
* ~~Support for multiple people in view, and zooming to keep them all in frame...~~


## Configuration Options

There are a handful of parameters that can be configured and passed to the container on launch via environment variables (through `-e`).

| Option            | Default        | Description |
| ----------------- | -------------- | ----------- |
| `VISCA_IP` (Required) | 192.168.1.134 | IP address of the PTZ camera. IP or hostname accepted |
| `VISCA_PORT `     | 52381          | Port that camera accepts VISCA commands on |
| `MQTT_HOST` (Required) | 10.1.1.175    | IP address of MQTT broker. IP or hostname accepted |
| `CONTROL  `       | False          | Whether to start controlling the camera automatically or wait for a start command. Use `True` or `False`. |
| `BOUNDARY `       | 0.35           | How far toward the screen edge can the person move before the camera starts following (default is 35% of screen size either side). 0 - 0.5 accepted. |
| `NET_RESOLUTION`  | -1x128         | Parameter sent directly to openpose. [See the Openpose documentation](https://github.com/CMU-Perceptual-Computing-Lab/openpose/blob/master/doc/demo_quick_start.md#improving-memory-and-speed-but-decreasing-accuracy). |
| `SHOW_UI`         | False          | Show the processed video in a window. Requires futher setup (see below). Use `True` or `False`. |
| `MIN_SPEED`       | 1              | Minimum panning speed for smooth accelerating. 0-23 accepted. Must be less than `MAX_SPEED` |
| `MAX_SPEED`       | 16             | Maximum panning speed for smooth accelerating. 1-24 accepted. Must me more than `MIN_SPEED` | 

## Companion Setup

If you have a StreamDeck running BitFocus Companion its straightforward to add a button to show and toggle the automatic control state.

* Add a new instance of "Generic MQTT".
* Configure it with Protocol: `mqtt://`, Broker IP: (As above, probably the control computer IP) and Port: `1883`.
* Add a new Regular Button.
* To show the camera state on the button:
  * In Instance Feedback add `mqtt: Change colors from MQTT topic value`.
  * Set the Topic to `PTZ_STATE`.
  * Set the Value to either `on` or `off` depending on what you want the button to show.
  * Set colours as desired.
* To turn the control on/off:
  * Add a new Key Down/On action of `mqtt: Publish Message`
  * Set the topic to `PTZ_SETSTATE`
  * Set the Payload to one of the following:

| Payload          | Description |
| ---------------- | ----------- |
| `control on`     | Turn automatic control on |
| `control off`    | Turn automatic control off |
| `control toggle` | Toggle the state of the automatic control |
| `control state`  | Force the program to republish the current control state (it does this automatically every 50 frames anyway)

![Compaion config screenshot](https://raw.githubusercontent.com/Cameron-D/openpose-ptz-control/main/Companion.png)

## Viewing the processed output

Nice to confirm what the program is actually seeing, but displaying windows from in Docker requires several changes (some which introduce possible security issues).

Open up X rendering to everyone (probably unsafe in untrusted environments):

```
$ xhost +
```

Launch the docker container with access to required host computer resources:

```
$ docker run --gpus all --name ptztrack \
    --rm -it \
    --net=host --ipc=host \
    -e DISPLAY=$DISPLAY \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -e VISCA_IP=10.1.1.174 \
    -e MQTT_HOST=10.1.1.175 \
    -e CONTROL=1 \
    -e SHOW_UI=1 \
    --device /dev/video0 ptztrack:11.1
```

After a moment a window should pop up with blue lines representing the movement edges, people marked in colours, a green box around the people and a blue point in the middle. If the blue point is outside the edges the camera will move in the required direction. Press Q to exit.

![Live preview screenshot](https://raw.githubusercontent.com/Cameron-D/openpose-ptz-control/main/Preview.png)
