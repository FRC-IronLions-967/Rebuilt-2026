// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  private IntakeIO io;
  private IntakeIOInputsAutoLogged inputs;
  private BooleanSupplier turretResetting;

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

  public Intake(IntakeIO io, BooleanSupplier turretResetting) {
    this.io = io;
    inputs = new IntakeIOInputsAutoLogged();
    this.turretResetting = turretResetting;
  }

  public Intake(IntakeIO io) {
    this(
      io,
      new BooleanSupplier() {
        @Override
        public boolean getAsBoolean() {
            return false;
        }
      }
    );
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
      case REVERSING:
        yield CurrentState.REVERSING;
    };
  }

  private void applyState() {
    switch (currentState) {
      case IDLE:
        stopAll();
        break;
      case INTAKING:
        intake(turretResetting.getAsBoolean());
        break;
      case REVERSING:
        reverse();
        break;
      default:
        stopAll();
        break;
    }
  }

  private void stopAll() {
    io.setIntakeArmAngle(IntakeConstants.armRestingPosition);
    io.setIntakeSpeed(0.0);
    io.setFeederSpeed(0.0);
    io.setHorizontal1Speed(0.0);
    io.setHorizontal2Speed(0.0);
  }

  private void intake(boolean resetting) {
    io.setIntakeArmAngle(IntakeConstants.intakePosition);
    io.setIntakeSpeed(IntakeConstants.intakeIntakingSpeed);
    io.setFeederSpeed(resetting ? 0 : IntakeConstants.feederSpeed);
    io.setHorizontal1Speed(IntakeConstants.horizontal1Speed);
    io.setHorizontal2Speed(IntakeConstants.horizontal2Speed);
  }

  private void reverse() {
    io.setIntakeArmAngle(IntakeConstants.intakePosition);
    io.setIntakeSpeed(-IntakeConstants.intakeIntakingSpeed);
    io.setFeederSpeed(-IntakeConstants.feederSpeed);
    io.setHorizontal1Speed(-IntakeConstants.horizontal1Speed);
    io.setHorizontal2Speed(-IntakeConstants.horizontal2Speed);
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public CurrentState getCurrentState() {
    return currentState;
  }
}