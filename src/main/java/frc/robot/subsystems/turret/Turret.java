// Copyright() (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Turret extends SubsystemBase {
  
  public final InterpolatingTreeMap<Double, ShooterSetpoint> shooterMap = new InterpolatingTreeMap<>(
    InverseInterpolator.forDouble(), 
    (start, end, t) ->
        new ShooterSetpoint(
            start.rpm + (end.rpm - start.rpm) * t,
            start.hoodAngle + (end.hoodAngle - start.hoodAngle) * t
        )

  );

  public final double shooterSetpointMinDistance = 0.0;//first key
  public final double shooterSetpointMaxDistance = 10.0;//max key

  public final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

  public final double TOFMinDistance = 0.0;//first key
  public final double TOFMaxDistance = 10.0;//max key

  public TurretIO io;
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
    HOMING
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  private double turretSetPoint = TurretConstants.turretMinAngle;
  private double hoodSetPoint = TurretConstants.hoodMinAngle;
  private double flywheelSetPoint = 0.0;
  private boolean homed = true;
  private boolean passing;

  double closestSafeAngle;

  /** Creates a new Turret. */
  public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    inputs = new TurretIOInputsAutoLogged();

    shooterMap.put(4.0, new ShooterSetpoint(5000, 9));
    shooterMap.put(3.0, new ShooterSetpoint(4000, 11));
    shooterMap.put(2.0, new ShooterSetpoint(3000, 13));
    shooterMap.put(1.0, new ShooterSetpoint(2000, 15));

    //sim entrys for testing DELETE
    timeOfFlightMap.put(1.0, 1.0);
    timeOfFlightMap.put(2.0, 2.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    currentState = updateState();
    applyState();
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret State", currentState);

    Logger.recordOutput("Passing?", passing);
  }

  
  /**
   * Updates the turret's current state based on the wanted state and homing status.
   *
   * <p>If the turret is homed, it matches the wanted state (IDLE or SHOOTING).
   * Otherwise, the turret transitions to HOMING until homed.
   *
   * @return the turret's current state.
   */
  private CurrentState updateState() {
    return switch(wantedState) {
      case IDLE:
        if (homed) {
          yield CurrentState.IDLE;
        }
        yield CurrentState.HOMING;
      case SHOOTING:
      if (homed) {
        yield CurrentState.SHOOTING;
      }
      yield CurrentState.HOMING;
    };
  }

  /**
   * Applies actions based on the turret's current state.
   *
   * <p>IDLE: Stops the flywheel, sets the hood to idle, and moves turret to the closest safe idle angle.
   * SHOOTING: Calculates target and sets flywheel, hood, and turret angles depending on robot position.
   * HOMING: Moves the turret until limit switches indicate it is homed.
   * DEFAULT: Stops flywheel, sets hood to idle, and keeps turret at current angle.
   */
  private void applyState() {
    switch (currentState) {
      case IDLE:
        io.setFlyWheelSpeed(0.0);
        io.setHoodAngle(TurretConstants.hoodIDLEPosition.get());
        closestSafeAngle = 
          Math.abs(TurretConstants.turretIDLEPosition2.get() - inputs.turretAngle) < Math.abs(TurretConstants.turretIDLEPosition1.get() - inputs.turretAngle) 
          ? TurretConstants.turretIDLEPosition2.get() 
          : TurretConstants.turretIDLEPosition1.get();
        io.setTurretAngle(closestSafeAngle);
        passing = false;
        break;
      case SHOOTING:
        if (false/*getPassingState(poseSupplier.get().getX(), TurretConstants.allianceZoneEnd())*/) {
          calculationToTarget(TurretConstants.hub());
          io.setFlyWheelSpeed(flywheelSetPoint);
          io.setHoodAngle(hoodSetPoint);
          io.setTurretAngle(turretSetPoint);
          passing = false;
        } else if (getPassingState(poseSupplier.get().getY(), TurretConstants.center())) {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          calculationToTarget(TurretConstants.right());
          io.setHoodAngle(TurretConstants.hoodPassingAngle.get());
          io.setTurretAngle(turretSetPoint);
          passing = true;
        } else {
          io.setFlyWheelSpeed(TurretConstants.flywheelPassingSpeed.get());
          calculationToTarget(TurretConstants.left());
          io.setHoodAngle(TurretConstants.hoodPassingAngle.get());
          io.setTurretAngle(turretSetPoint);
          passing = true;
        }
        break;
      case HOMING:
        homed = io.home();
        passing = false;
        break;
      default:
        io.setFlyWheelSpeed(0.0);
        io.setHoodAngle(TurretConstants.hoodIDLEPosition.get());
        io.setTurretAngle(inputs.turretAngle);
        passing = false;
        break;
    }
  }

  /**
   * Sets the desired turret state.
   *
   * @param wantedState the state to transition to
   */
  public void setWantedState(WantedState wantedState) {
    this.wantedState = wantedState;
  }

  /**
   * Adjusts a target position to compensate for the robot's motion.
   * Iteratively accounts for changes in time-of-flight (TOF) caused by the robot moving.
   *
   * @param target the original target position
   * @return the adjusted target position considering robot velocity
   */ 
  public Translation2d considerChassisSpeeds(Translation2d target) {
    double TOF = 0;
    double previousTOF = 0;
    ChassisSpeeds speeds = speedsSupplier.get();
    for (int i = 0; i < 3; i++) {
      previousTOF = TOF;
      TOF = timeOfFlightMap.get(MathUtil.clamp(target.getDistance(poseSupplier.get().getTranslation()), TOFMinDistance, TOFMaxDistance));
      target = new Translation2d(
        target.getX() - speeds.vxMetersPerSecond * (TOF - previousTOF),
        target.getY() - speeds.vyMetersPerSecond * (TOF - previousTOF));
    }
    return target;
  }

  /**
   * Calculates and sets the turret and hood setpoints to aim at a given target.
   * Considers the robot's motion and interpolates shooter settings based on distance.
   *
   * @param target the position to aim at
   */
  private void calculationToTarget(Translation2d target) {
    Translation2d adjustedTarget = considerChassisSpeeds(target);
    Translation2d robotToTarget = adjustedTarget.minus(poseSupplier.get().getTranslation());
    Rotation2d turretToTargetAngle = robotToTarget.getAngle().minus(poseSupplier.get().getRotation());
    turretSetPoint = MathUtil.angleModulus(turretToTargetAngle.getRadians());
    Logger.recordOutput("Calculations/target", adjustedTarget);
    Logger.recordOutput("Calculations/robotToTarget", robotToTarget);
    Logger.recordOutput("Calculations/turretToTargetAngle", turretToTargetAngle);
    //calculate hood angle based off distance
    double distance = robotToTarget.getNorm();
    ShooterSetpoint setpoint = shooterMap.get(MathUtil.clamp(distance, shooterSetpointMinDistance, shooterSetpointMaxDistance));
    hoodSetPoint = MathUtil.clamp(setpoint.hoodAngle, TurretConstants.hoodMinAngle, TurretConstants.hoodMaxAngle);
    flywheelSetPoint = MathUtil.clamp(setpoint.rpm, 0, 6758);
  }

  private boolean getPassingState(double test, double line) {
    if (DriverStation.getAlliance().get() == Alliance.Blue) {
      return test < line;
    }
    return line < test;
  }

  /**
   * Returns the current state of the turret.
   *
   * @return the turret's current state
   */
  public CurrentState getCurrentState() {
    return currentState;
  }

  /**
   * Checks if the turret is currently resetting (moving across a large angle).
   *
   * @return true if the turret is resetting, false otherwise
   */
  public boolean getResetting() {
    return inputs.resetting;
  }

  /**
   * Checks if the turret is in a safe position for the intake to operate.
   *
   * @return true if intake operation is safe, false otherwise
   */
  public boolean intakeSafe() {
    // return inputs.intakeSafe;
    return true;
  }

  /**
   * Checks if the flywheel has reached a speed high enough for shooting.
   *
   * @return true if the flywheel speed exceeds the minimum running speed, false otherwise
   */
  public boolean shooterSpedUp() {
    return inputs.flywheelSpeed > TurretConstants.flywheelMinRunningSpeed.get();
  }

  /**
   * Returns the current angle of the hood.
   *
   * @return the hood angle in radians
   */
  public double getHoodAngle() {
    return inputs.hoodAngle;
  }
}
