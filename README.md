[![CI](https://github.com/BHSRobotix/ChargedUp2023/actions/workflows/main.yml/badge.svg)](https://github.com/BHSRobotix/ChargedUp2023/actions/workflows/main.yml)

# Charged Up 2023
Code for the FRC 2023 Robotics Competition

## Resources
* [Official FIRST Robotics Competition Control System (WPILib)](https://docs.wpilib.org/en/stable/)
  * [WPILib Installation Guide](https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html)

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