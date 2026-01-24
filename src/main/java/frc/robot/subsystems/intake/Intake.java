// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.intake.IntakeIO.IntakeIOInputs;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputs inputs;

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

  public Intake(IntakeIO io) {
    this.io = io;
    inputs = new IntakeIOInputs();
    }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake", (LoggableInputs) inputs);
    currentState = updateState();
    applyState();
  }

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
        io.setIntakeSpeed(5600);
        io.setFeederSpeed(5600);
        io.setHorizontalMotor1Speed(5600);
        io.setHorizontalMotor2Speed(5600);
        break;
      case REVERSING:
        io.setIntakeArmAngle(IntakeConstants.intakePosition);
        io.setIntakeSpeed(-5600);
        io.setFeederSpeed(-5600);
        io.setHorizontalMotor1Speed(-5600);
        io.setHorizontalMotor2Speed(-5600);
        break;
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
