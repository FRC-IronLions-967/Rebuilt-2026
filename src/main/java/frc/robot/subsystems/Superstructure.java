// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.vision.ObjectDetectionVision;

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
    RESETTINGTURRET,
    EJECTING
  }

  private WantedState wantedState;
  private CurrentState currentState;

  private Drive drive;
  private AprilTagVision aprilTagVision;
  private ObjectDetectionVision objectDetectionVision;
  private Turret turret;

  public Superstructure(Drive drive, AprilTagVision aprilTagVision, ObjectDetectionVision objectDetectionVision, Turret turret) {
    this.drive = drive;
    this.aprilTagVision = aprilTagVision;
    this.objectDetectionVision = objectDetectionVision;
    this.turret = turret;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState = updateState(wantedState);
  }

  private CurrentState updateState(WantedState wantedState) {
    return switch (wantedState) {
      case IDLE:
        yield CurrentState.IDLE;
      case SHOOTING:
        if (turret.getCurrentState() == Turret.CurrentState.RESETTING) {
          yield CurrentState.RESETTINGTURRET;
        }
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
      //APRILTAG IDS still need set and camera index
        turret.setWantedState(Turret.WantedState.SHOOTING, aprilTagVision.getTargetInfo(0, 0).targetRot().getX(), aprilTagVision.getTargetInfo(0, 0).distanceToTarget());
        //intake->intaking
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
