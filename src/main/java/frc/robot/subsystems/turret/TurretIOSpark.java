// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

// import java.util.function.BooleanSupplier;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// import frc.robot.util.LimitSwitchManager;

/** Add your docs here. */
public class TurretIOSpark implements TurretIO{

    protected SparkFlex flywheel;
    protected SparkFlexConfig flywheelConfig;

    protected SparkFlex flywheelFollower;
    protected SparkFlexConfig flywheelFollowerConfig;

    protected SparkFlex hood;
    protected SparkFlexConfig hoodConfig;
    protected SparkClosedLoopController hoodController;

    protected BangBangController flyWheelBangBang;
    protected SimpleMotorFeedforward flywheelFeedforward;

    // protected SparkFlex turret;
    // protected SparkFlexConfig turretConfig;
    // protected SparkClosedLoopController turretController;

    protected double flywheelSetSpeed;
    protected double hoodSetAngle;
    // protected double turretSetAngle;

    // protected BooleanSupplier turretMinLimitSwitch;
    // protected BooleanSupplier turretMaxLimitSwitch;

    public TurretIOSpark() {
        flywheel = new SparkFlex(9, MotorType.kBrushless);
        flywheelConfig = new SparkFlexConfig();
        flywheelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(TurretConstants.flywheelCurrentLimit);
        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flyWheelBangBang = new BangBangController(TurretConstants.flywheelTolerance.get());
        flywheelFeedforward = new SimpleMotorFeedforward(TurretConstants.flywheelkS.get(), TurretConstants.flywheelkV.get(), TurretConstants.flywheelkA.get());

        flywheelFollower = new SparkFlex(10, MotorType.kBrushless);
        flywheelFollowerConfig = new SparkFlexConfig();
        flywheelFollowerConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(TurretConstants.flywheelCurrentLimit).follow(flywheel);
        flywheelFollower.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        hood = new SparkFlex(11, MotorType.kBrushless);
        hoodConfig = new SparkFlexConfig();
        hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(TurretConstants.hoodCurrentLimit).closedLoop.pid(TurretConstants.hoodP.get(), 0.0, TurretConstants.hoodD.get()).feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(TurretConstants.hoodMinAngle, TurretConstants.hoodMaxAngle);
        hoodConfig.encoder.positionConversionFactor(1/36);// gear ratio is 12:1 then <3 rotations from top to bottom
        hood.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodController = hood.getClosedLoopController();

        hood.getEncoder().setPosition(TurretConstants.hoodMinAngle);

        // turret = new SparkFlex(12, MotorType.kBrushless);
        // turretConfig = new SparkFlexConfig();
        // turretConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(TurretConstants.turretCurrentLimit).closedLoop.pid(TurretConstants.turretP.get(), 0, TurretConstants.turretD.get()).feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(TurretConstants.turretMinAngle, TurretConstants.turretMaxAngle).positionWrappingEnabled(false);
        // turretConfig.encoder.positionConversionFactor(2*Math.PI * TurretConstants.turretGearRatio);
        // turret.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // turretController = turret.getClosedLoopController();

        // turretMinLimitSwitch = LimitSwitchManager.getSwitch(0);
        // turretMaxLimitSwitch = LimitSwitchManager.getSwitch(1);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // if (turretMinLimitSwitch.getAsBoolean()) {
        //     turret.getEncoder().setPosition(TurretConstants.turretMinAngle);
        // } else if (turretMaxLimitSwitch.getAsBoolean()) {
        //     turret.getEncoder().setPosition(TurretConstants.turretMaxAngle);
        // }

        inputs.flywheelSpeed = flywheel.getEncoder().getVelocity();
        inputs.flywheelSetSpeed = flywheelSetSpeed;
        inputs.hoodAngle = hood.getAbsoluteEncoder().getPosition();
        // inputs.turretAngle = turret.getEncoder().getPosition();
        // inputs.turretSetAngle = turretSetAngle;
        // inputs.turretMinLimitSwitch = turretMinLimitSwitch.getAsBoolean();
        // inputs.turretMaxLimitSwitch = turretMaxLimitSwitch.getAsBoolean();
        // inputs.resetting = Math.abs(inputs.turretAngle - inputs.turretSetAngle) > Math.PI;
        // inputs.intakeSafe = 
        //     ((TurretConstants.turretIDLEPosition1.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition1.get() + Math.PI/4)
        //     || ((TurretConstants.turretIDLEPosition2.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition2.get() + Math.PI/4)));
        
        flywheel.setVoltage(flyWheelBangBang.calculate(flywheel.getEncoder().getVelocity(), flywheelSetSpeed) * 12 + 0.9 * flywheelFeedforward.calculate(flywheelSetSpeed));
        hoodController.setSetpoint(hoodSetAngle, ControlType.kPosition);
        // turretController.setSetpoint(turretSetAngle, ControlType.kPosition);
    }

    @Override
    public void setFlyWheelSpeed(double speed) {
        flywheelSetSpeed = speed;
    }

    @Override
    public void setHoodAngle(double angle) {
        hoodSetAngle = angle;
    }

    // /**
    //  * loops through itself until it makes the angle become an angle a valid angle
    //  * @param angle wanted angle
    //  */
    // @Override
    // public void setTurretAngle(double angle) {
    //     if (angle < TurretConstants.turretMinAngle) {
    //         if (angle + Math.PI*2 > TurretConstants.turretMaxAngle) {
    //             if (TurretConstants.turretMinAngle - angle <= angle + Math.PI*2 - TurretConstants.turretMaxAngle) {
    //                 setTurretAngle(TurretConstants.turretMinAngle);
    //             } else {
    //                 setTurretAngle(TurretConstants.turretMaxAngle);
    //             }
    //         } else {
    //             setTurretAngle(angle + Math.PI*2);
    //         }
    //     } else if (angle > TurretConstants.turretMaxAngle ) {
    //         if (angle - Math.PI*2 < TurretConstants.turretMinAngle) {
    //             if (angle - TurretConstants.turretMaxAngle <= TurretConstants.turretMinAngle - angle - Math.PI*2) {
    //                 setTurretAngle(TurretConstants.turretMaxAngle);
    //             } else {
    //                 setTurretAngle(TurretConstants.turretMinAngle);
    //             }
    //         } else {
    //             setTurretAngle(angle - Math.PI * 2);
    //         }
    //     } else {
    //         turretSetAngle = angle;
    //     }
    // }
}
