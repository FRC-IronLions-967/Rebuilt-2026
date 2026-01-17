// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.VisionConstants;

public class Superstructure extends SubsystemBase {
  /** Creates a new Superstructure. */
  public enum WantedState {
    IDLE,
    SHOOTING,
    EJECTING
  }

  public enum CurrentState {
    IDLE,
    SHOOTING,
    PASSING,
    RESETTINGTURRET,
    EJECTING
  }

  private WantedState wantedState;
  private CurrentState currentState;

  private Drive drive;
  private AprilTagVision aprilTagVision;
  private Turret turret;

  private Translation2d passingTarget;
  private double passingTurretRot;

  public Superstructure(Drive drive, AprilTagVision aprilTagVision, Turret turret) {
    this.drive = drive;
    this.aprilTagVision = aprilTagVision;
    this.turret = turret;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState = updateState(wantedState);
    applyState();
  }

  private CurrentState updateState(WantedState wantedState) {
    return switch (wantedState) {
      case IDLE:
        yield CurrentState.IDLE;
      case SHOOTING:
        if (turret.getCurrentState() == Turret.CurrentState.RESETTING) {
          yield CurrentState.RESETTINGTURRET;
        } else if (drive.getPose().getX() > 4.5) yield CurrentState.PASSING;
        yield CurrentState.SHOOTING;
      case EJECTING:
        yield CurrentState.EJECTING;
    };
  }

  private void applyState() {
    switch (currentState) {
      case IDLE:
        turret.setWantedState(Turret.WantedState.IDLE);
        //intake->IDLE
        break;
      case SHOOTING:
        turret.setWantedState(Turret.WantedState.SHOOTING, aprilTagVision.getTargetInfo(VisionConstants.turretCameraIndex, VisionConstants.hubAprilTag).targetYaw(), aprilTagVision.getTargetInfo(VisionConstants.turretCameraIndex, VisionConstants.hubAprilTag).distanceToTarget());
        //intake->intaking
        break;
      case PASSING:
        passingTarget = drive.getPose().getX() > 4 ? new Translation2d(1, 7) : new Translation2d(1, 1);
        passingTurretRot = passingTarget.minus(drive.getPose().getTranslation()).getAngle().minus(drive.getRotation()).getRadians();
        turret.setWantedState(Turret.WantedState.SHOOTING, passingTurretRot, drive.getPose().getTranslation().getDistance(passingTarget));
        //intake-intaking
        break;
      case RESETTINGTURRET:
        turret.setWantedState(Turret.WantedState.SHOOTING);
        //intake->idle
        break;
      case EJECTING:
        turret.setWantedState(Turret.WantedState.SHOOTING);
        //intake->reversing
      default:
        turret.setWantedState(Turret.WantedState.IDLE);
        //intake->IDLE
        break;
    }
  }

  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  public Command setWantedStateCommand(WantedState wantedState) {
    return new InstantCommand(() -> setWantedState(wantedState));
  }
}
