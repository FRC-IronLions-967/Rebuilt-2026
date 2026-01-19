// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs;

  private Supplier<Pose2d> poseSupplier;

  public enum WantedState {
    IDLE,
    SHOOTING,
  }

  public enum CurrentState {
    IDLE,
    SHOOTING,
    RESETTING
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  private boolean hitMax;

  private PIDController turretController = new PIDController(TurretConstants.turretControllerP.get(), 0, TurretConstants.turretControllerD.get());

  /** Creates a new Turret. */
  public Turret(TurretIO io, Supplier<Pose2d> poseSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    inputs = new TurretIOInputsAutoLogged();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
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
        if (poseSupplier.get().getX() < 4.5) {
          io.setFlyWheelSpeed(TurretConstants.flywheelShootingSpeed.get());
          io.setHoodAngle(getHoodAngleBasedOnDistance(TurretConstants.hub));
          io.setTurretAngle(turretController.calculate(getFieldOrientedTurretRotation2d(inputs.turretAngle).getRadians(), getRobotToTargetAngle(TurretConstants.hub).getRadians()));
        } else if (poseSupplier.get().getY() < 4) {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          io.setHoodAngle(getHoodAngleBasedOnDistance(TurretConstants.right));
          io.setTurretAngle(turretController.calculate(getFieldOrientedTurretRotation2d(inputs.turretAngle).getRadians(), getRobotToTargetAngle(TurretConstants.right).getRadians()));
        } else {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          io.setHoodAngle(getHoodAngleBasedOnDistance(TurretConstants.left));
          io.setTurretAngle(turretController.calculate(getFieldOrientedTurretRotation2d(inputs.turretAngle).getRadians(), getRobotToTargetAngle(TurretConstants.left).getRadians()));
        }
        break;
      default:
        io.setFlyWheelSpeed(0.0);
        io.setHoodAngle(TurretConstants.hoodIDLEPosition.get());
        io.setTurretAngle(inputs.turretAngle);
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

  //Make table
  private double getHoodAngleBasedOnDistance(Translation2d target) {
    double distance = poseSupplier.get().getTranslation().getDistance(target);
    //do something with distance
    return Math.PI;
  }

  private Rotation2d getRobotToTargetAngle(Translation2d target) {
    Translation2d robotToTargetTranslation = target.minus(poseSupplier.get().getTranslation());
    return Rotation2d.fromRadians(Math.atan(robotToTargetTranslation.getY()/robotToTargetTranslation.getX()));
  }

  private Rotation2d getFieldOrientedTurretRotation2d(double robotRelativeAngle) {
    return Rotation2d.fromRadians(robotRelativeAngle).plus(poseSupplier.get().getRotation());
  }

  public CurrentState getCurrentState() {
    return currentState;
  }
}
