// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs;

  public enum WantedState {
    IDLE,
    SHOOTING
  }

  private enum CurrentState {
    IDLE,
    SHOOTING,
    RESETTING
  }

  private WantedState wantedState;
  private CurrentState currentState;

  private boolean hitMax;
  private Rotation2d targetYawRot;
  private double targetDistance;

  /** Creates a new Turret. */
  public Turret(TurretIO io) {
    this.io = io;
    inputs = new TurretIOInputsAutoLogged();
    TurretConstants.turretAngleController.enableContinuousInput(TurretConstants.turretMinAngle, TurretConstants.turretMaxAngle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    currentState = updateState();
    applyState();
  }

  private CurrentState updateState() {
    return switch(wantedState) {
      case IDLE: 
        yield CurrentState.IDLE;
      case SHOOTING:
        if ((currentState == CurrentState.RESETTING && ((hitMax && inputs.turretAngle > 0) || (!hitMax && inputs.turretAngle < 0))) 
        || (inputs.turretSetAngle > TurretConstants.turretMaxAngle || inputs.turretSetAngle < TurretConstants.turretMinAngle)) {
          yield CurrentState.RESETTING;
        } else {
          yield CurrentState.SHOOTING;
        }
    };
  }

  private void applyState() {
    switch (currentState) {
      case IDLE:
        io.setFlyWheelSpeed(0.0);
        io.setHoodAngle(TurretConstants.hoodIDLEPosition.get());
        io.setTurretAngle(inputs.turretAngle);
        break;
      case RESETTING:
        if (inputs.turretAngle > 0) {
          hitMax = true;
          io.setTurretAngle(-.1);
        } else {
          hitMax = false;
          io.setTurretAngle(0.1);
        }
        break;
      case SHOOTING:
        io.setFlyWheelSpeed(TurretConstants.flywheelShootingSpeed.get());
        io.setTurretAngle(TurretConstants.turretAngleController.calculate(targetYawRot.getRadians(), 0));
        io.setHoodAngle(getHoodAngleBasedOnDistance(targetDistance));
        break;
      default:
        break;
    }
  }

  /**
   * Sets the turret's state
   * @param wantedState what state to set to.
  */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * Should be updated periodically if we are shooting
   * @param turretRotationSetpoint yaw of the target
   * @param hoodDistanceSetpoint distance to plug into a map
   */
  public void updateTurretTarget (Rotation2d turretRotationSetpoint, double hoodDistanceSetpoint) {
    this.targetYawRot = turretRotationSetpoint;
    this.targetDistance = hoodDistanceSetpoint;
  }

  //Make table
  private double getHoodAngleBasedOnDistance(double distance) {
    if (distance < 1) {
      return Math.PI/2;
    } else if (distance >= 1 && distance < 2) {
      return Math.PI*3/8;
    }
    return Math.PI/4;
  }
}
