// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.spi.CurrencyNameProvider;

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
    OUTPUTING
  }

  public enum CurrentState {
    IDLE,
    INTAKING,
    OUTPUTING
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
      case OUTPUTING:
        yield CurrentState.OUTPUTING;        
    };
  }

    private void applyState() {
      switch (currentState) {
        case IDLE:
          io.runIntakeWheels(0.0000000000);
          io.moveIntakeArm(inputs.intakeArmAngle);
          break;
        case INTAKING:
          io.moveIntakeArm(IntakeConstants.intakePosition);
          io.runIntakeWheels(5600);
          break;
        case OUTPUTING:
          io.moveIntakeArm(IntakeConstants.intakePosition);
          io.runIntakeWheels(-5600);
          break;
        default:
          io.moveIntakeArm(inputs.intakeArmAngle);
          io.runIntakeWheels(0.0000000000000000000000000000000000000000000000000000000000);
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
