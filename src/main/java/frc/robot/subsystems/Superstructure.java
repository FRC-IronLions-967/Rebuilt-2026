// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.turret.Turret;
import frc.robot.subsystems.vision.AprilTagVision;
import frc.robot.subsystems.intake.Intake;

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
    EJECTING
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  @SuppressWarnings("unused")
  private Drive drive;
  @SuppressWarnings("unused")
  private AprilTagVision aprilTagVision;
  private Turret turret;
  private Intake intake;

  public Superstructure(Drive drive, AprilTagVision aprilTagVision, Turret turret, Intake intake) {
    this.drive = drive;
    this.aprilTagVision = aprilTagVision;
    this.turret = turret;
    this.intake = intake;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState = updateState(wantedState);
    applyState();
    
    Logger.recordOutput("Wanted State", wantedState);
    Logger.recordOutput("CurrentState", getCurrentState());
  }

  /**
   * Maps a desired (wanted) state to the corresponding current state.
   *
   * @param wantedState the desired subsystem state
   * @return the corresponding current state
   */
  private CurrentState updateState(WantedState wantedState) {
    return switch (wantedState) {
      case IDLE:
        yield CurrentState.IDLE;
      case SHOOTING:
        yield CurrentState.SHOOTING;
      case EJECTING:
        yield CurrentState.EJECTING;
    };
  }

  /**
   * Applies actions to the turret and intake subsystems based on the current state.
   * Updates the wanted state of each subsystem depending on the overall system state.
   */
  private void applyState() {
    switch (currentState) {
      case IDLE:
        turret.setWantedState(Turret.WantedState.IDLE);
        if (turret.intakeSafe()) {
          intake.setWantedState(Intake.WantedState.IDLE);
        }
        break;
      case SHOOTING:
        turret.setWantedState(Turret.WantedState.SHOOTING);
        intake.setWantedState(Intake.WantedState.INTAKING);
        break;
      case EJECTING:
        turret.setWantedState(Turret.WantedState.SHOOTING);
        intake.setWantedState(Intake.WantedState.REVERSING);
        break;
      default:
        turret.setWantedState(Turret.WantedState.IDLE);
        intake.setWantedState(Intake.WantedState.IDLE);
        break;
    }
  }

  /**
   * Returns the current state of the superstructure.
   *
   * @return the current state
   */
  public CurrentState getCurrentState() {
    return currentState;
  }

  /**
   * Sets the desired state for the subsystem.
   *
   * @param wantedState the state to set
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * Returns a command that sets the subsystem's wanted state instantly.
   *
   * @param wantedState the state to set
   * @return an InstantCommand that sets the wanted state
   */
  public Command setWantedStateCommand(WantedState wantedState) {
    return new InstantCommand(() -> setWantedState(wantedState));
  }
}
