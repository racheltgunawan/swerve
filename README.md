# Swerve Drive for Cedar Park Robotics

### Overview
This repository contains my practice code for the Swerve drive on our competition robot for Cedar Park Robotics team CPR 3663. As one of the software developers of the Team, I was in charge of working on the Swerve drive. This code is written in Java and utilizes the WPI library and other classes pulled from hardware documentation.

### What is swerve
Our robot utilizes Swerve drive which is the best alternative from tank drive. Swerve allows each wheel of the robot to move independently, allowing the robot to turn 360 degrees effortlessly. This is done using trig to calculate the necessary angles for each wheel and calculating the speed needed for a smooth rotation using a PID controller.

### The Snap Code
I also created a snap rotation code that allows the robot to move to the nearest 45 degree angle.
1. Takes in the input from the Xbox controller to see which angle it would like to automatically snap to:
      - Called in RobotContainer.class so that the code can repeatedly check for input from the controller
2. Calculates the current angle and how much time + what angle the robot needs to rotate at.
      - Instantiated in C_Drive.class and C_SnapRotate.class
  
### Challenges
My main challenge throughout this project was being able to deciper the code of the team who created Swerve drive. Because Swerve drive was created by a different team and bought by us, no documentation was written for us to understand the code. Therefore, I had to read lots of online documentation regarding the public functions that they used from the modules. After this task, I became well-versed in reading and analyzing code and its functions.
