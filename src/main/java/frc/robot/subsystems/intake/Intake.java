// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  public IntakeIO io;
  private IntakeIOInputsAutoLogged inputs;
  private BooleanSupplier turretResetting;
  private BooleanSupplier flyWheelSpedUp;

  public enum WantedState {
    IDLE,
    INTAKING,
    REVERSING
  }

  public enum CurrentState {
    IDLE,
    INTAKING,
    REVERSING
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  public Intake(IntakeIO io, BooleanSupplier turretResetting, BooleanSupplier flyWheelSpedUp) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    this.turretResetting = turretResetting;
    this.flyWheelSpedUp = flyWheelSpedUp;
  }

  public Intake(IntakeIO io) {
    this(io, ()-> false, ()-> true);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    currentState = updateState();
    applyState();
  }

  /**
   * Determines the current state of the intake subsystem based on the wanted state.
   *
   * @return the corresponding CurrentState
   */
  private CurrentState updateState() {
    return switch(wantedState) {
      case IDLE:
        yield CurrentState.IDLE;
      case INTAKING:
        yield CurrentState.INTAKING;
      case REVERSING:
        yield CurrentState.REVERSING;
    };
  }

  /**
   * Executes actions based on the current intake state.
   * Stops motors when IDLE, runs intake when INTAKING, and reverses when REVERSING.
   */
  private void applyState() {
    switch (currentState) {
      case IDLE:
        stopAll();
        break;
      case INTAKING:
        intake(turretResetting.getAsBoolean(), flyWheelSpedUp.getAsBoolean());
        break;
      case REVERSING:
        reverse();
        break;
      default:
        stopAll();
        break;
    }
  }

  /**
   * Stops all intake-related motors and moves the arm to its resting position.
   */
  private void stopAll() {
    io.setIntakeArmAngle(IntakeConstants.armRestingPosition);
    io.setIntakeSpeed(0.0);
    io.setFeederSpeed(0.0);
    io.setHorizontal1Speed(0.0);
    io.setHorizontal2Speed(0.0);
  }

  /**
   * Runs the intake system to collect game pieces.
   * 
   * @param resetting true if the turret is resetting, in which case the feeder is stopped
   * @param flywheelSpedUp true if the flywheel is at speed, allowing the feeder to run
   */
  private void intake(boolean resetting, boolean flywheelSpedUp) {
    io.setIntakeArmAngle(IntakeConstants.intakePosition);
    io.setIntakeSpeed(IntakeConstants.intakeIntakingSpeed);
    io.setFeederSpeed(resetting || !flywheelSpedUp ? 0 : IntakeConstants.feederSpeed);
    io.setHorizontal1Speed(IntakeConstants.horizontal1Speed);
    io.setHorizontal2Speed(IntakeConstants.horizontal2Speed);
  }

  /**
   * Runs the intake system in reverse to eject game pieces.
   */
  private void reverse() {
    io.setIntakeArmAngle(IntakeConstants.intakePosition);
    io.setIntakeSpeed(-IntakeConstants.intakeIntakingSpeed);
    io.setFeederSpeed(-IntakeConstants.feederSpeed);
    io.setHorizontal1Speed(-IntakeConstants.horizontal1Speed);
    io.setHorizontal2Speed(-IntakeConstants.horizontal2Speed);
  }

  /** 
   * Sets the desired state of the intake subsystem.
   *
   * @param wantedState the state to transition to
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * Returns the current state of the intake subsystem.
   *
   * @return the current state
   */
  public CurrentState getCurrentState() {
    return currentState;
  }
}
