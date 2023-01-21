# Desired Robot Controls

## Hardware
- 2 Joysticks
    - 1 dedicated for driving
    - 1 dedicated for arm/gripper controls

## Functionality
### Driver Control
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
- Sensors
    - NavX
    - Ultrasonic (for detecting when hanging off ramp?)
    - Visual Camera
    - PhotonVision Camera?

### Arm Control
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
- Sensors
    - Limit Switch
        - Upper
        - Lower
    - Visual Camera?
