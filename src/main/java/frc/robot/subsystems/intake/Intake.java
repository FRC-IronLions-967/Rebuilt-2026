// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs;

  public enum WantedState {
    IDLE,
    INTAKING,
    RESETTING,
    REVERSING
  }

  public enum CurrentState {
    IDLE,
    INTAKING,
    RESETTING,
    REVERSING
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", inputs);
    currentState = updateState();
    applyState();
  }

  private CurrentState updateState() {
    return switch(wantedState) {
      case IDLE:
        yield CurrentState.IDLE;
      case INTAKING:
        yield CurrentState.INTAKING;
      case RESETTING:
        yield CurrentState.RESETTING;
      case REVERSING:
        yield CurrentState.REVERSING;
    };
  }

  private void applyState() {
    switch (currentState) {
      case IDLE:
        io.setIntakeArmAngle(IntakeConstants.armRestingPosition);
        io.setIntakeSpeed(0.0);
        io.setFeederSpeed(0.0);
        io.setHorizontalMotor1Speed(0.0);
        io.setHorizontalMotor2Speed(0.0);
        break;
      case INTAKING:
        io.setIntakeArmAngle(IntakeConstants.intakePosition);
        io.setIntakeSpeed(IntakeConstants.intakeIntakingSpeed);
        io.setFeederSpeed(IntakeConstants.feederSpeed);
        io.setHorizontalMotor1Speed(IntakeConstants.horizontal1Speed);
        io.setHorizontalMotor2Speed(IntakeConstants.horizontal2Speed);
        break;
      case REVERSING:
        io.setIntakeArmAngle(IntakeConstants.intakePosition);
        io.setIntakeSpeed(-IntakeConstants.intakeIntakingSpeed);
        io.setFeederSpeed(-IntakeConstants.feederSpeed);
        io.setHorizontalMotor1Speed(-IntakeConstants.horizontal1Speed);
        io.setHorizontalMotor2Speed(-IntakeConstants.horizontal2Speed);
        break;
      case RESETTING:
        //stop feeding but keep intaking
        io.setIntakeArmAngle(IntakeConstants.intakePosition);
        io.setIntakeSpeed(-IntakeConstants.intakeIntakingSpeed);
        io.setFeederSpeed(0);
        io.setHorizontalMotor1Speed(-IntakeConstants.horizontal1Speed);
        io.setHorizontalMotor2Speed(-IntakeConstants.horizontal2Speed);
      default:
        io.setIntakeArmAngle(IntakeConstants.armRestingPosition);
        io.setIntakeSpeed(0.0);
        io.setFeederSpeed(0.0);
        io.setHorizontalMotor1Speed(0.0);
        io.setHorizontalMotor2Speed(0.0);
        break;
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public CurrentState getCurrentState() {
    return currentState;
  }
}
