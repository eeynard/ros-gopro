#Ros-gopro

This project is fully tested with:
* Virtual Box 5.0.9
* Virtual Box Extension Pack 5.0.9
* Vagrant 1.7.4

Retrieve the preview of the gopro does not work on a vm with Ubuntu

## How to run the whole app
1. Upload arduino/gimbal-controller/gimbal-controller.ino on your Arduino board
2. Connect your gimbal to the arduino
  * Ground wire (black for gimbal feiyu G3) on arduino's ground
  * Pitch wire (white for gimbal feiyu G3) on pin 2
  * Heading wire (brown for gimbal feiyu G3) on pin 7
  * Mode wire (yellow for gimbal feiyu G3) on pin 4
3. Connect your gimbal to battery
4. Connect your computer to the gopro's wifi
5. Finally launch the main launch file
```
roslaunch gopro ros_gopro.launch
```

## GoPro Controller

### How it works

The GoPro is controlled via HTTP calls defined by the constructor. It permits to enable the gopro, getting its status, ...
 
### Getting the status

* **ROS Topic:**
  * /gopro/status

You can get all the status of the GoPro on this topic, refreshed every second. 
It is retrieved by making several calls to the HTTP Api and merging 
responses to one message.
 
### Retrieve images

* **ROS Topic:**
  * /gopro/camera/picture
  * /gopro/camera/take_picture
            
Sending Int32 1 to topic /gopro/camera/take_picture, the ROS node publish a sensor_msgs.msg.Image to the /gopro/camera/picture topic.

            
### Troubleshooting

We can activate a preview mode on the GoPro which makes a stream available on udp://@10.5.5.9:8554.
As the preview is using significant energy, we have to send keep alive packet every 2.5 seconds maximum.

We can retrieve video data with FFPlay:
```
ffplay -fflags nobuffer -f:v mpegts -probesize 8192 udp://@10.5.5.9:8554
```

OpenCV seems to be unable to retrieve UDP video stream over network. 
Retrieving a frame with FFMpeg is possible but it is really slow. 
It is due to that PPS frame are sent only one time each 5 / 10 seconds in the MPEGTS format and those frames
are essential to FFMpeg to decode and save a frame to an image.

So, as a work-around of those issues, we decided to take a picture with the GoPro, retrieve it from HTTP and sending it to the ROS Node

## Gimbal
The gimbal is controlled by an arduino UNO using ROS and rosserial.

### Prerequisite
In order to be able to run this app you should first follow this quick [tutorial](http://wiki.ros.org/rosserial_arduino/Tutorials) on how to install rosserial and how to integrate it in your Arduino IDE. (I supposed you already have a working installation of ROS, if not check the [installation instructions](http://wiki.ros.org/indigo/Installation))

### Upload code to your board
Open the ros-gopro/arduino/gimbal-controller.ino file in your Arduino IDE. It will tell you that you need to create a new sketch on your sketchbook and will offer you to do it for you. Just say yes :)

Now you can compile the code and upload the binary to the board.

### Launch the ROS node
At this point, your arduino is ready to communicate with the gimbal. Next step is to launch the ROS node that will subscribe to several topics to control the axes. To do so, first launch roscore in a new terminal :
```
roscore
```
Then open another terminal and launch the rosserial node that will execute our code from the arduino :
```
rosrun rosserial_python serial_node.py _port:=/dev/ttyACM0 # Put the right COM port here (it can be ttyUSB0, ttyUSB1, ...)
```
Now our ROS node is waiting for new data on 3 differents topics :
* RGP\_pitch\_control : [std_msgs/UInt16] Use this topic to rotate your camera on the horizontal axis
* RGP\_heading\_control : [std_msgs/UInt16] Use this topic rotate the camera on the vertical axis
* RGP\_mode\_switch : [std_msgs/UInt16] Use this topic to configure the gimbal mode
  * 1 = Heading Follow Mode：Camera Pitch and Roll angles remain constant, heading follows the nose position,pitch can be control by RC
  * 2 = Heading And Pitch Follow Mode：Camera Roll angle remains constant. Heading follow the nose position and Pitch follow the elevation of the aircreft.
  * 3 = Heading Lock Mode: Heading, Pitch and Roll are all locked to point at one position.Heading and pitch can be control by RC

### Send data to the topics
Last thing to do is to publish to ROS topics to control the gimbal. It is a ROS compliant app so you can use the basic way to publish data on ROS. If you want to do a quick test you can use ROS command line :
```
rostopic pub RGP_pitch_control std_msgs/UInt16 60
```
## GoPro Analyzer
### Aim
Detect faces in Go Pro pictures, compute distance between camera and faces, and publish face positions (in degrees). 
### Prerequisites
Rect python library is needed
```
pip install rect
```
### How to launch it
```
roslaunch gopro_analyzer analyzer.launch
```
### How it works
GoPro Analyzer listen GoPro Controller topics which send pictures and camera information (such as video resolution - 16/9 or 4/3 - and fov - camera horizontal angle - in degrees). It defines camera vertical angle with those information.

GoPro Analyzer uses OpenCV to detect faces and eyes ([tutorial](http://docs.opencv.org/master/d7/d8b/tutorial_py_face_detection.html#gsc.tab=0)), and then determines faces position on pictures and in front of the camera by using camera v/h angles.

It also compute distance between camera and faces by using measures took by hand (face widths in pixels are proportionnal to the face distances in centimeter with camera, so we can approximate it for any face widths after we took some measures).

In the end, GoPro Analyzer publish all this information in topics (see below).

GoPro Analyzer launcher launch RVIZ where raw pictures and analyzed pictures are displayed.

### Topics
#### Subscriber
/gopro/camera/picture (sensor_msgs.msg import Image) GoPro camera pictures

/gopro/status (gopro.msg import Status) GoPro camera statuses
#### Publisher
/gopro/camera/take_picture (std_msgs.msg import Int64) GoPro command to take pictures

/analyzer/picture/h_angle (std_msgs.msg import Float64) GoPro camera horizontal angle

/analyzer/picture/raw (sensor_msgs.msg import Image) GoPro camera raw pictures

/analyzer/picture/vidRes (std_msgs.msg import Float64) GoPro camera picture resolutions

/analyzer/picture/v_angle (std_msgs.msg import Float64) GoPro camera vertical angle

/analyzer/face/position (gopro_analyzer.msg import FacePosition) Face positions in degress

/analyzer/picture/analyzed (sensor_msgs.msg import Image) Analyzed picture

/analyzer/face/distance (std_msgs.msg import Float64) Face distances from camera


### Messages
#### FacePosition
```
float64 vertical # positive or negative values
float64 horizontal # positive or negative values
```
