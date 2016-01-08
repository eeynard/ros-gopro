#Ros-gopro

This project is fully tested with:
* Virtual Box 5.0.9
* Virtual Box Extension Pack 5.0.9
* Vagrant 1.7.4

Retrieve the preview of the gopro does not work on a vm with Ubuntu

## GoPro Controller

### How it works

The GoPro is controlled via HTTP calls defined by the constructor. It permits to enable the gopro, getting its status, ...
 
### Getting the status

**ROS Topic:**
    - /gopro/status

You can get all the status of the GoPro on this topic, refreshed every second. 
It is retrieved by making several calls to the HTTP Api and merging 
responses to one message.
 
### Retrieve images

**ROS Topic:**
    - /gopro/camera/picture
    - /gopro/camera/take_picture
            
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

