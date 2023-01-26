## Drive Train Subsystem Structure

### High Level Structure/Plans:
- Odomotry and Kinematics (Pose Estimation)
    - Integrate Photon Vision
    - Encoders
    - NavX
    - Field2d
- PID based drive
    - Voltage based drive
- Basic drive
    - Arcade drive
- Autonomous:
  - Driver assistance methods:
    - TODO:

### Methods:
arcadeDrive(double speed, double rotation) - drive the robot using arcade drive
resetEncoders() - reset the encoders to 0
resetNavX() - reset the NavX  to 0
setupTalons() - sets up the talons for the drive train

### Getters:
getLeftMaster() - get the left master motor
getRightMaster() - get the right master motor
getLeftFollower() - get the left follower motor
getRightFollower() - get the right follower motor
getNavX() - get the navX


### Setters:
setTalonMode(NeutralMode mode) - set the talon mode
