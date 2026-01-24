// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.turret.TurretIO.TurretIOInputs;


public class Turret extends SubsystemBase {
  private TurretIO io;
  private TurretIOInputs inputs;

  private Supplier<Pose2d> poseSupplier;

  public enum WantedState {
    IDLE,
    SHOOTING
  }

  public enum CurrentState {
    IDLE,
    SHOOTING
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  /** Creates a new Turret. */
  public Turret(TurretIO io, Supplier<Pose2d> poseSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    inputs = new TurretIOInputs();
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
        yield CurrentState.SHOOTING;
    };
  }

  private void applyState() {
    switch (currentState) {
      case IDLE:
        io.setFlyWheelSpeed(0.0);
        io.setHoodAngle(TurretConstants.hoodIDLEPosition.get());
        io.setTurretAngle(inputs.turretAngle);
        break;
      case SHOOTING:
        if (poseSupplier.get().getX() < 4.5) {
          io.setFlyWheelSpeed(TurretConstants.flywheelShootingSpeed.get());
          io.setHoodAngle(getHoodAngleBasedOnDistance(TurretConstants.hub));
          io.setFieldRelativeTurretAngle(getRobotToTargetAngle(TurretConstants.hub), poseSupplier.get().getRotation());
        } else if (poseSupplier.get().getY() < 4) {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          io.setHoodAngle(getHoodAngleBasedOnDistance(TurretConstants.right));
          io.setFieldRelativeTurretAngle(getRobotToTargetAngle(TurretConstants.right), poseSupplier.get().getRotation());
        } else {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          io.setHoodAngle(getHoodAngleBasedOnDistance(TurretConstants.left));
          io.setFieldRelativeTurretAngle(getRobotToTargetAngle(TurretConstants.left), poseSupplier.get().getRotation());
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
    Translation2d robotToTarget = target.minus(poseSupplier.get().getTranslation());
    return robotToTarget.getAngle();
  }

  public CurrentState getCurrentState() {
    return currentState;
  }

  public boolean getResetting() {
    return inputs.resetting;
  }
}
