package bhs.devilbotz.subsystems.stubs;

import bhs.devilbotz.subsystems.Arm;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ArmStub extends Arm {
  private double position = 0;
  private double speed = 10; /* delta per period */

  private enum ArmState {
    STOPPED,
    MOVING_UP,
    MOVING_DOWN
  }

  ArmState state = ArmState.STOPPED;

  public ArmStub() {
    state = ArmState.STOPPED;
  }

  @Override
  public void periodic() {
    switch (state) {
      case MOVING_UP:
        position += speed;
        System.out.println("Arm Up " + position);
        break;

      case MOVING_DOWN:
        position -= speed;
        System.out.println("Arm Down " + position);
        break;

      default:
        break;
    }
    SmartDashboard.putNumber("Arm Position", position);
  }

  @Override
  public void setSpeed(double speed) {
    this.speed = speed;
  }

  @Override
  public void up() {
    state = ArmState.MOVING_UP;
  }

  @Override
  public void down() {
    state = ArmState.MOVING_DOWN;
  }

  @Override
  public void stop() {
    state = ArmState.STOPPED;
  }

  @Override
  public boolean isMoving() {
    return (ArmState.STOPPED != state);
  }

  @Override
  public boolean isTopLimit() {
    return (position >= topPosition + 10);
  }

  @Override
  public boolean isBottomLimit() {
    return (position <= 0);
  }

  @Override
  public double getPosition() {
    return position;
  }

  @Override
  protected void resetPosition() {
    position = 0;
  }
}
