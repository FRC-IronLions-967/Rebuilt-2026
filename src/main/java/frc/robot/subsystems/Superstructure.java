// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.util.Color;
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

  private double matchTime;
  private Color autoWinColor;
  private String gameData;
  private double periodTimer = 0;

  private Alliance previousAlliance = Alliance.Blue;

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
    Logger.recordOutput("FieldBasedTurret", turret.getTurretAngle()-drive.getPose().getRotation().getRadians());
    matchTime = DriverStation.getMatchTime();
    gameData = DriverStation.getGameSpecificMessage();

    Logger.recordOutput("Hub Active", updateHubStatusAndPeriod(matchTime, gameData));
    Logger.recordOutput("Match Time", matchTime);
    Logger.recordOutput("Period Time", periodTimer);
    Logger.recordOutput("Auto Winner", autoWinColor);

    if (previousAlliance != DriverStation.getAlliance().orElse(previousAlliance)) {
      turret.redoPassingFunction();
    } 
    previousAlliance = DriverStation.getAlliance().orElse(previousAlliance);
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

  public boolean updateHubStatusAndPeriod(double matchTime, String gameData) {
    Optional<Alliance> alliance = DriverStation.getAlliance();

    // Default autoWinColor
    autoWinColor = new Color(0,0,0);

    // Update autoWinColor based on game data
    if (!gameData.isEmpty()) {
        switch (gameData.charAt(0)) {
            case 'B' -> autoWinColor = new Color(0, 0, 255);
            case 'R' -> autoWinColor = new Color(255, 0, 0);
            default -> autoWinColor = new Color(); // fallback
        }
    } else {
        autoWinColor = new Color(); // fallback if empty
    }

    // No alliance? hub inactive.
    if (alliance.isEmpty()) {
        periodTimer = 0;
        return false;
    }

    // Autonomous: hub always active, period = remaining time in auto
    if (DriverStation.isAutonomousEnabled()) {
        periodTimer = matchTime; // seconds left in auto period
        return true;
    }

    // Not teleop enabled? hub inactive
    if (!DriverStation.isTeleopEnabled()) {
        periodTimer = 0;
        return false;
    }

    boolean redInactiveFirst = switch (gameData.isEmpty() ? ' ' : gameData.charAt(0)) {
        case 'R' -> true;
        case 'B' -> false;
        default -> false; // fallback
    };

    boolean shift1Active = switch (alliance.get()) {
        case Red -> !redInactiveFirst;
        case Blue -> redInactiveFirst;
    };

    // Determine hub status and period timer based on match time
    boolean hubActive;
    if (matchTime > 130) {
        hubActive = true;
        periodTimer = matchTime - 130; // seconds left until shift 1
    } else if (matchTime > 105) {
        hubActive = shift1Active;
        periodTimer = matchTime - 105; // shift 1 timer
    } else if (matchTime > 80) {
        hubActive = !shift1Active;
        periodTimer = matchTime - 80; // shift 2 timer
    } else if (matchTime > 55) {
        hubActive = shift1Active;
        periodTimer = matchTime - 55; // shift 3 timer
    } else if (matchTime > 30) {
        hubActive = !shift1Active;
        periodTimer = matchTime - 30; // shift 4 timer
    } else {
        hubActive = true;
        periodTimer = matchTime; // endgame
    }

    return hubActive;
  }
}
