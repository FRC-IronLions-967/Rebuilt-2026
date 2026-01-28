// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {
  
  public static final InterpolatingDoubleTreeMap hoodMap = new InterpolatingDoubleTreeMap();
  public static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  private TurretIO io;
  private TurretIOInputsAutoLogged inputs;

  private Supplier<Pose2d> poseSupplier;
  private Supplier<ChassisSpeeds> speedsSupplier;

  public enum WantedState {
    IDLE,
    SHOOTING
  }

  public enum CurrentState {
    IDLE,
    SHOOTING,
    TRENCH
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  private double turretSetPoint = TurretConstants.turretMinAngle;
  private double hoodSetPoint = TurretConstants.hoodMinAngle;

  double closestSafeAngle;

  /** Creates a new Turret. */
  public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    inputs = new TurretIOInputsAutoLogged();

    // hoodMap.put(null, null); to add hood made shots
    // timeOfFlightMap.put(null, null); add time of flight mesurments

    //sim entrys for testing DELETE
    timeOfFlightMap.put(1.0, 1.0);
    timeOfFlightMap.put(2.0, 2.0);
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
        for (int i = 0; i < TurretConstants.trenches.length; i++) {
            if (TurretConstants.trenches[i].getDistance(poseSupplier.get().getTranslation()) < TurretConstants.turretTolerance) {
              yield CurrentState.TRENCH;
            }
          }
        yield CurrentState.SHOOTING;
    };
  }

  private void applyState() {
    switch (currentState) {
      case IDLE:
        io.setFlyWheelSpeed(0.0);
        io.setHoodAngle(TurretConstants.hoodIDLEPosition.get());
        closestSafeAngle = 
          Math.abs(TurretConstants.turretIDLEPosition2.get() - inputs.turretAngle) < Math.abs(TurretConstants.turretIDLEPosition1.get() - inputs.turretAngle) 
          ? TurretConstants.turretIDLEPosition1.get() 
          : TurretConstants.turretIDLEPosition2.get();
        io.setTurretAngle(closestSafeAngle);
        break;
      case SHOOTING:
        if (poseSupplier.get().getX() < 4.5) {
          io.setFlyWheelSpeed(TurretConstants.flywheelShootingSpeed.get());
          calculationToTarget(TurretConstants.hub);
          io.setHoodAngle(hoodSetPoint);
          io.setTurretAngle(turretSetPoint);
        } else if (poseSupplier.get().getY() < 4) {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          calculationToTarget(TurretConstants.right);
          io.setHoodAngle(hoodSetPoint);
          io.setTurretAngle(turretSetPoint);
        } else {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          calculationToTarget(TurretConstants.left);
          io.setHoodAngle(hoodSetPoint);
          io.setTurretAngle(turretSetPoint);
        }
        break;
      case TRENCH:
        io.setFlyWheelSpeed(0);
        io.setHoodAngle(TurretConstants.turretMinAngle);
        if (poseSupplier.get().getX() < 4.5) {
          calculationToTarget(TurretConstants.hub);
          io.setTurretAngle(turretSetPoint);
        } else if (poseSupplier.get().getY() < 4) {
          calculationToTarget(TurretConstants.right);
          io.setTurretAngle(turretSetPoint);
        } else {
          calculationToTarget(TurretConstants.left);
          io.setTurretAngle(turretSetPoint);
        }
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

  /**
   * Use this method to account for the robot's velocity
   * Loops through TOF times a couple times to account for the fact that we are changing the target so the TOF will change
   * @param target origional wanted position
   * @return the target modified to account for the robot's velocity
   */
  public Translation2d considerChassisSpeeds(Translation2d target) {
    double TOF = 0;
    double previousTOF = 0;
    for (int i = 0; i < 3; i++) {
      previousTOF = TOF;
      TOF = timeOfFlightMap.get(target.getDistance(target));
      target = new Translation2d(
        target.getX() - speedsSupplier.get().vxMetersPerSecond * (TOF - previousTOF),
        target.getY() - speedsSupplier.get().vyMetersPerSecond * (TOF - previousTOF));
    }
    return target;
  }

  /**
   * 
   * @param target where the turret should be aimed at hood and turret
   */
  private void calculationToTarget(Translation2d target) {
    Translation2d robotToTarget = considerChassisSpeeds(target).minus(poseSupplier.get().getTranslation());
    Rotation2d turretToTargetAngle = robotToTarget.getAngle().minus(poseSupplier.get().getRotation());
    turretSetPoint = turretToTargetAngle.getRadians();
    Logger.recordOutput("Calculations/target", considerChassisSpeeds(target));
    Logger.recordOutput("Calculations/robotToTarget", robotToTarget);
    Logger.recordOutput("Calculations/turretToTargetAngle", turretToTargetAngle);
    //calculate hood angle based off distance
  }

  public CurrentState getCurrentState() {
    return currentState;
  }

  public boolean getResetting() {
    return inputs.resetting;
  }
  public boolean intakeSafe() {
    return inputs.intakeSafe;
  }
}
