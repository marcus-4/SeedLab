This is our code for the final demo.

We targeted the same deliverables as demo2, so our code is a more refined version. 

Our robot raspberry pi identifies the aruco marker angle and distance, and communicates the distance to the arduino

The control system approaches the marker and draws a loop around it

It is integrated and works. 

this code uses the same camera calibration files from demo 1 under/demoone/

We did further calibration and speed adjustment, with the resulting modified files in FinalDemo_Backup and FinalDemo_BackupSpeed in the repository. They are the same base files

We will further discuss our changes and approach in the presentation. One issue we faced was problems detecting the marker upon rotation of the robot, due to camera blur.


| Team Member | Task                                            |
|-------------:|:-------------------------------------------------:|
| Landon      | Tuning the controller and Arduino code          |
| Rose        | Mechanical Assembly of the robot                 |
| Marcus      | Tuning detection parameters                      |
| Stow        | Integrating I2C communication to Arduino        |
| All         | Debugging and refinement.                       |



 Source file index

 | Name             | Description                           |
|------------------:|:---------------------------------------:|
|AudioStuff.h       | Motion beep header                   |
| FinalDemo.ino      | Arduino project code                  |
| I2CStuff.h       | Arduino I2C header                    |
| Motor.h          | Arduino motor header                  |
| RobotParams.h    | Arduino robot parameters header       |
| FinalDemo.py         | Raspberry Pi detection code           |

