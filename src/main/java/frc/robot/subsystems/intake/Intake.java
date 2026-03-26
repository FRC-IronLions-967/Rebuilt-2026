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
  private int jamCount = 0; 
  private int unjamCount = 0;
  private boolean jammed = false;

  public enum WantedState {
    IDLE,
    PAUSED,
    INTAKING,
    REVERSING,
    TESTING
  }

  public enum CurrentState {
    IDLE,
    PAUSED,
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
    if (wantedState != WantedState.INTAKING) {
      jamCount = 0;
      unjamCount = 0;
      jammed = false;
    }

    return switch(wantedState) {
      case IDLE:
        yield CurrentState.IDLE;
      case INTAKING:
        if (inputs.intakeCurrent > IntakeConstants.jamCurrent && inputs.intakeSpeed < IntakeConstants.jamSpeed && !jammed) {
          jamCount++;
        } else {
          jamCount = 0;
        }
        if (jamCount >= IntakeConstants.jamMinCount) {
          jammed = true;
        }
        if (jammed) {
          unjamCount++;
          if (unjamCount >= IntakeConstants.unjamMinCount) {
            jammed = false;
            jamCount = 0;
            unjamCount = 0;
          }
          yield CurrentState.REVERSING;
        }
        yield CurrentState.INTAKING;
      case PAUSED:
        yield CurrentState.PAUSED;
      case REVERSING:
        yield CurrentState.REVERSING;
      case TESTING:
        yield CurrentState.INTAKING;//running withou unjam
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
      case PAUSED:
      pause();
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
    io.stopIntake();
    io.setFeederSpeed(0.0);
    io.setHorizontal1Speed(0.0);
    io.setHorizontal2Speed(0.0);
  }

  private void pause() {
    io.stopIntake();
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

  public boolean getRumble() {
    return jammed;
  }

  public double getTotalCurrent() {
    return inputs.subsystemCurrent;
  }

  public boolean armOut () {
    return Math.abs(inputs.armAngle - inputs.armSetAngle) < 0.25;
  }
}
