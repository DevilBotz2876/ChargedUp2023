[![CI](https://github.com/BHSRobotix/ChargedUp2023/actions/workflows/main.yml/badge.svg)](https://github.com/BHSRobotix/ChargedUp2023/actions/workflows/main.yml)
[![Javadoc](https://img.shields.io/badge/JavaDoc-Online-green)](https://bhsrobotix.github.io/ChargedUp2023/javadoc/)

# Charged Up 2023
Code for the FRC 2023 Robotics Competition

## Resources
* [Official FIRST Robotics Competition Control System (WPILib)](https://docs.wpilib.org/en/stable/)
    * [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)
* API Documentation
    * [WPILib API](https://github.wpilib.org/allwpilib/docs/release/java/index.html)
    * [Talon SRX Motor Controller](https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/package-summary.html)
    * [NavX Gyro Sensor](https://www.kauailabs.com/public_files/navx-mxp/apidocs/java/com/kauailabs/navx/frc/package-summary.html)

## Hardware Configuration
- DriveTrain
    - Motors
        - Left
            - Master: Talon SRX (on CAN ID 3)
            - Follower: Talon SRX (on CAN ID 1)
        - Right:
            - Master: Talon SRX (on CAN ID 4) - Inverted
            - Follower: Talon SRX (on CAN ID 2) - Inverted  
    - Gear Box
        - Toughbox Mini Classic
            - Counts Per Revolution: 4096 
            - Encoder Ratio: 1:1
        - Wheels
            - 6" Diameter (0.1524 meters)
    - Gyro
        - NavX (on SPI MXP Port)
- Arm
    - Motor
        - TBD
    - Limit Switches
        - Upper
            - TBD
        - Lower
            - TBD
- Gripper
    - TBD
- Cameras
    - TBD

## Desired Robot Controls

### Hardware
- 2 Joysticks
    - 1 dedicated for driving
    - 1 dedicated for arm/gripper controls

### Functionality
#### Driver Control
- Stick
    - arcade drive w/ proportional speed
    - y-axis is forward/back
    - x-axis is turn left/right
- Buttons
    - Driver Assistance (to be discussed/designed further)
        - scoringZone (use april tag?)
            - left
            - middle
            - right
        - if pieceMode == cone: (use reflective tape?)
            - left
            - right
        - activate balance
- Robot Hardware
  - Sensors
      - NavX
      - Ultrasonic (for detecting when hanging off ramp?)
      - Visual Camera
      - PhotonVision Camera? (placement on robot is critical for effective vision)

#### Arm Control
- Stick
    - Y-axis w/ proportional speed
        - forward = arm down
        - backward = arm up
- Buttons
    - gripControl
        - open
        - close
    - pieceSelection (will set LED on robot accordingly and adjust gripper pressure)
        - cone
        - cube
    - placementPosition (dependent on pieceMode)
        - low
        - medium
        - high
    - pickupPosition (dependent on pieceMode?)
        - shelf
        - ground (cube or upright cone)
        - ground (flat cone)
- Hardware
  - Sensors
      - Limit Switch
          - Upper
          - Lower
      - Visual Camera?
  - LED Light (to indicate cone vs cube pickup to human @ loading station)

## Desired Autonomous Routines
The routines in **bold** are the most important/common routines we will use

1. Routine 1: Sit still
1. Routine 2: Mobility
    1. Drive straight forwards towards the middle of the field
    * [Parameters](#Parameters-Definitions)
        * Delay
        * Distance
1. **Routine 3: Score and Mobility**
    1. Score preloaded game piece
    1. Drive straight forwards towards the middle of the field
    * [Parameters](#Parameters-Definitions)
        * Delay
        * Distance
        * ScoreLevel
1. Routine 4: Dock and engage
    1. Drive straight onto charge station and balance
    1. Use PID to automatically balance
    * [Parameters](#Parameters-Definitions)
        * Delay
1. **Routine 5: Mobility, dock, and engage**
    1. Drive straight forwards towards the middle of the field over the charge station
    1. Drive back onto the charge station
    1. Use PID to automatically balance
    * [Parameters](#Parameters-Definitions)
        * Delay
        * Distance
1. Routine 6: Score, dock, and engage
    1. Score preloaded game piece
    1. Drive straight onto charge station and balance
    1. Use PID to automatically balance
    * [Parameters](#Parameters-Definitions)
        * Delay
        * ScoreLevel
1. **Routine 7: Score, mobility, dock, and engage**
    1. Score preloaded game piece
    1. Drive straight forwards towards the middle of the field over the charge station
    1. Drive back onto the charge station
    1. Use PID to automatically balance
    * [Parameters](#Parameters-Definitions)
        * Delay
        * Distance
        * ScoreLevel
1. Routine 8: Score, mobility, pick up a game piece from center field, dock, and engage
    1. Score preloaded game piece
    1. Drive straight forwards towards the middle of the field over the charge station
    1. Pick up a game piece at center field
    1. Drive back onto the charge station
    1. Use PID to automatically balance
    * [Parameters](#Parameters-Definitions)
        * Delay
        * Distance
        * ScoreLevel
        * GamePieceType
### Parameters Definitions
    Delay: Delay before robot starts moving, in seconds
    Distance: Distance to leave the community, in feet/meters (TBD)
    ScoreLevel: Either low, medium, or high
    GamePieceType: Either cone or cube

