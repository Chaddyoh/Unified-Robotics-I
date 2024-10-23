# Unified Robotics I
From January 2023 - March 2023, the completion of the second robotics course at WPI required the creation of a 2-wheeled robot with a custom made four-bar. 

## Description
The robot is programmed to autonomously pick up **Solar Panels** and place them upon varying **Roofs** or remove **Solar Panels** from **Roofs**. **Solar Panels** are 101mm x 101mm x6mm plates of either wood, plastic, or aluminum. Each solar panel would have two 25.75mm diameter holes in them to allow placement on **Roofs**. **Roofs** where slanted wooden boards at 25&deg; slope and 45&deg; slope. On each &**Roof** would be two pegs that the **Solar Panels** would fit onto.

<img src="img/Solar Panel.jpg" alt="drawing" width="350"/>

*Example of wooden Solar Panel.*

<img src="img/final_demo_cropped.gif" alt="drawing" width="350"/>

*GIF of final demo, robot is placing Solar Panel on Roof.*

## Mechanical Designing
When designing the four-bar, the three point method was used between the gripper at flat state, gripper at angle for 25&deg; slope, and gripper at angle for 45&deg; slope. This step is waht dictates the length of the two four-bar pieces that are not the gripper and the two mounting points for the four-bar.

After, the gripper design was generated in CAD to hold a continuous servo which would push a piece down until a certain torque threshold meaning that the gripper has successfully grabbed a **Solar Panel**. The gripper was then 3D-printed and attached to the four-bar. The four-bar with the specified dimensions above was then designed with CAD and laser-cut for assembly.

<img src="img/CAD Gripper and four-bar.png" alt="drawing" width="500"/>

*CAD Desgin of both gripper and four-bar that was mounted on top of our robot.*

## Code Design
One of the design requirements was to handle a manual emergency stop from an IR remote. To accomplish this, a rudementary sequence was designed alongside our state machine to allow storing which step the program was at so when the state is switched via emergency stop, it can swtich back to its correct state and sequence.

``` cpp
if (emergencyStop) {
    eStopHappened = true;
    chassis.idle();
    servo.writeMicroseconds(servoStop);
    motor.setEffort(0);
    eStopLeftState = initialLeftRotation;
    eStopRightState = initialRightRotation;
    turning = false;
  }
```
*Handling when emergency stop is pressed. Seen in main.cpp, line 837 - 845.*

To navigate the field, reflectance sensors were utilized with Bang Bang control to follow the path represented with black tape on a white surfaceand the turns were proportionally controlled with the built-in IMU sensor. To control the four-bar, motor encoders were used and the gripper on the end of the four-bar was controlled using a continuous servo.

``` cpp
void lineFollower(bool direction) {
  int leftRead = analogRead(LineTrackerPin3);
  int rightRead = analogRead(LineTrackerPin1);

  int leftMotorSpeed = -100;
  int rightMotorSpeed = -110;
  if ((leftRead > 500) && (rightRead < 400)) {
    rightMotorSpeed += 20;
  } else if ((leftRead < 400) && (rightRead > 500)) {
    leftMotorSpeed += 20;
  }
  if (!(direction)) {
    leftMotorSpeed = leftMotorSpeed * -1;
    rightMotorSpeed = rightMotorSpeed * -1;
  }
  chassis.setMotorEfforts(leftMotorSpeed, rightMotorSpeed);
}
```
*Function for following line. Seen in main.cpp, line 72 - 88.*