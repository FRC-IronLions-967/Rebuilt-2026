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
import frc.robot.util.AllianceFlipUtil;


public class Turret extends SubsystemBase {
  
  private final InterpolatingTreeMap<Double, ShooterSetpoint> shooterShootingMap = new InterpolatingTreeMap<>(
    InverseInterpolator.forDouble(), 
    (start, end, t) ->
        new ShooterSetpoint(
            start.rpm + (end.rpm - start.rpm) * t,
            start.hoodAngle + (end.hoodAngle - start.hoodAngle) * t
        )

  );

  private final InterpolatingTreeMap<Double, ShooterSetpoint> shooterPassingMap = new InterpolatingTreeMap<>(
    InverseInterpolator.forDouble(), 
    (start, end, t) ->
        new ShooterSetpoint(
            start.rpm + (end.rpm - start.rpm) * t,
            start.hoodAngle + (end.hoodAngle - start.hoodAngle) * t
        )

  );

  private final InterpolatingDoubleTreeMap shooterFullFieldMap = new InterpolatingDoubleTreeMap();

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
    SHOOTING,
    TESTING
  }

  public enum CurrentState {
    IDLE,
    SHOOTING,
    PASSING,
    FULLFIELD,
    HOMING,
    TESTING
  }

  private WantedState wantedState = WantedState.IDLE;
  private CurrentState currentState = CurrentState.IDLE;

  private double turretSetPoint = TurretConstants.turretMinAngle;
  private double hoodSetPoint = TurretConstants.hoodMinAngle;
  private double flywheelSetPoint = 0.0;
  private boolean homed = true;
  private Pose2d pose;

  double closestSafeAngle;

  //Optimization variables to decrease runtime
  private Translation2d adjustedTarget;
  private Translation2d robotToTarget;
  private Rotation2d turretToTargetAngle;
  private ChassisSpeeds speeds;
  private ShooterSetpoint setpoint;
  private ShooterSetpoint passingSetpoint;

  /** Creates a new Turret. */
  public Turret(TurretIO io, Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> speedsSupplier) {
    this.io = io;
    this.poseSupplier = poseSupplier;
    this.speedsSupplier = speedsSupplier;
    inputs = new TurretIOInputsAutoLogged();

    shooterShootingMap.put(1.225, new ShooterSetpoint(2200, 0.573 + TurretConstants.hoodOffset));
    shooterShootingMap.put(1.869, new ShooterSetpoint(2250, 0.573 + TurretConstants.hoodOffset));
    shooterShootingMap.put(3.28, new ShooterSetpoint(2750, 0.5 + TurretConstants.hoodOffset));
    shooterShootingMap.put(4.91, new ShooterSetpoint(3000, 0.377 + TurretConstants.hoodOffset));
    shooterShootingMap.put(0.916, new ShooterSetpoint(2000, 0.614 + TurretConstants.hoodOffset));
    shooterShootingMap.put(2.25, new ShooterSetpoint(2500, 0.614 + TurretConstants.hoodOffset));
    shooterShootingMap.put(4.231, new ShooterSetpoint(2750, 0.51 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(3.01, new ShooterSetpoint(2500, 0.51 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(3.70, new ShooterSetpoint(2600, 0.435 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(4.952, new ShooterSetpoint(3000, 0.36 + TurretConstants.hoodOffset));

    
    //This is how we are going to tune the shots. This is how 6328 did it
    // shooterShootingMap.put(idk, new ShooterSetpoint(2000, 0.614 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(2125, 0.60 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(2250, 0.575 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(2375, 0.55 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(2500, 0.525 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(2625, 0.50 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(2750, 0.475 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(2875, 0.450 + TurretConstants.hoodOffset));
    // shooterShootingMap.put(idk, new ShooterSetpoint(3000, 0.425 + TurretConstants.hoodOffset));





    //need to entrys for the passing/fullfield
    shooterPassingMap.put(TurretConstants.allianceZoneEnd(), TurretConstants.startNZ);
    shooterPassingMap.put(TurretConstants.oppositeAllianceEnd(), TurretConstants.endNZ);
    shooterFullFieldMap.put(TurretConstants.oppositeAllianceEnd(), TurretConstants.endNZ.rpm);
    shooterFullFieldMap.put(TurretConstants.fieldEnd(), TurretConstants.endFullField.rpm);

    timeOfFlightMap.put(1.805, 1.008);
    timeOfFlightMap.put(2.97, 1.378);
    timeOfFlightMap.put(3.35, 2.043);
    timeOfFlightMap.put(4.890, 1.433);
    // timeOfFlightMap.put(0.0, 0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    pose = poseSupplier.get();
    Logger.recordOutput("Turret Pose", pose);
    currentState = updateState();
    applyState();
    io.updateInputs(inputs);
    Logger.processInputs("Turret", inputs);
    Logger.recordOutput("Turret State", currentState);
    Logger.recordOutput("DistanceToHub", pose.getTranslation().getDistance(TurretConstants.hub()));
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
        if (!homed) {
          yield CurrentState.HOMING;
        } else if(isPastLine(pose.getX(), TurretConstants.allianceZoneEnd())) {
          yield CurrentState.SHOOTING;
        } else if(isPastLine(pose.getX(), TurretConstants.oppositeAllianceEnd())) {
          yield CurrentState.PASSING;
        }
        yield CurrentState.FULLFIELD;
      case TESTING:
        yield CurrentState.TESTING;
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
        io.stopTurret();
        break;
      case SHOOTING:
        calculationToTarget(TurretConstants.hub());
        io.setFlyWheelSpeed(flywheelSetPoint);
        io.setHoodAngle(hoodSetPoint);
        io.setTurretAngle(turretSetPoint);
        break;
      case PASSING:
        passingSetpoint = shooterPassingMap.get(pose.getX());
        calculationToTarget(chooseTargetBasedOnY(pose.getTranslation(), TurretConstants.left(), TurretConstants.right(), TurretConstants.center()));
        io.setFlyWheelSpeed(passingSetpoint.rpm);
        io.setHoodAngle(passingSetpoint.hoodAngle);
        io.setTurretAngle(turretSetPoint);
        break;
      case FULLFIELD:
        calculationToTarget(chooseTargetBasedOnY(pose.getTranslation(), TurretConstants.left(), TurretConstants.right(), TurretConstants.center()));
        double rpm = shooterFullFieldMap.get(AllianceFlipUtil.applyX(pose.getX()));
        io.setFlyWheelSpeed(rpm);
        io.setHoodAngle(TurretConstants.endFullField.hoodAngle);
        io.setTurretAngle(turretSetPoint);
        break;
      case HOMING:
        homed = true;
        break;
      case TESTING:
        io.setFlyWheelSpeed(TurretConstants.testingFlywheelSpeed.get());
        io.setHoodAngle(TurretConstants.testingHoodAngle.get());
        calculationToTarget(TurretConstants.hub());
        io.setTurretAngle(turretSetPoint);
        break;
      default:
        io.setFlyWheelSpeed(0.0);
        io.setHoodAngle(TurretConstants.hoodIDLEPosition.get());
        io.stopTurret();;
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
    speeds = speedsSupplier.get();
    ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(speeds, pose.getRotation());
    for (int i = 0; i < 3; i++) {
      previousTOF = TOF;
      TOF = timeOfFlightMap.get(MathUtil.clamp(target.getDistance(pose.getTranslation()), TOFMinDistance, TOFMaxDistance)) * TurretConstants.ToFRealityConstant.get();
      target = new Translation2d(
        target.getX() - fieldRelativeSpeeds.vxMetersPerSecond * (TOF - previousTOF),
        target.getY() - fieldRelativeSpeeds.vyMetersPerSecond * (TOF - previousTOF));
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
    adjustedTarget = considerChassisSpeeds(target);
    robotToTarget = adjustedTarget.minus(pose.getTranslation());
    turretToTargetAngle = robotToTarget.getAngle().minus(pose.getRotation());
    turretSetPoint = MathUtil.angleModulus(turretToTargetAngle.getRadians());
    Logger.recordOutput("Calculations/target", adjustedTarget);
    // Logger.recordOutput("Calculations/robotToTarget", robotToTarget);
    // Logger.recordOutput("Calculations/turretToTargetAngle", turretToTargetAngle);
    //calculate hood angle based off distance
    double distance = robotToTarget.getNorm();
    setpoint = shooterShootingMap.get(MathUtil.clamp(distance, shooterSetpointMinDistance, shooterSetpointMaxDistance));
    hoodSetPoint = MathUtil.clamp(setpoint.hoodAngle, TurretConstants.hoodMinAngle, TurretConstants.hoodMaxAngle);
    flywheelSetPoint = MathUtil.clamp(setpoint.rpm, 0, 6758);
  }

  private boolean isPastLine(double robotX, double lineX) {
    boolean isRed = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    if (isRed) {
        return robotX > lineX;
    } else {
        return robotX < lineX;
    }
  }

  private Translation2d chooseTargetBasedOnY(
    Translation2d robotPose,
    Translation2d left,
    Translation2d right,
    double centerY) {

    boolean isRed =
        DriverStation.getAlliance().orElse(Alliance.Blue)
        == Alliance.Red;

    if (isRed) {
        return robotPose.getY() > centerY ? right : left;
    } else {
        return robotPose.getY() > centerY ? left : right;
    }
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
    return inputs.intakeSafe;
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

  public double getTurretAngle() {
    return inputs.turretAngle;
  }

  public void redoPassingFunction() {
    shooterPassingMap.clear();
    shooterFullFieldMap.clear();

    shooterPassingMap.put(TurretConstants.allianceZoneEnd(), TurretConstants.startNZ);
    shooterPassingMap.put(TurretConstants.oppositeAllianceEnd(), TurretConstants.endNZ);
    shooterFullFieldMap.put(TurretConstants.oppositeAllianceEnd(), TurretConstants.endNZ.rpm);
    shooterFullFieldMap.put(TurretConstants.fieldEnd, TurretConstants.endFullField.rpm);
  }
}
