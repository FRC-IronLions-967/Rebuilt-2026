// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Add your docs here. */
public class TurretIOSpark implements TurretIO{

    protected SparkFlex flywheel;
    protected SparkFlexConfig flywheelConfig;
    protected SparkClosedLoopController flywheelController;

    protected SparkFlex hood;
    protected SparkFlexConfig hoodConfig;
    protected SparkClosedLoopController hoodController;

    protected SparkFlex turret;
    protected SparkFlexConfig turretConfig;
    protected SparkClosedLoopController turretController;

    protected double flywheelSetSpeed;
    protected double hoodSetAngle;
    protected double turretSetAngle;

    public TurretIOSpark() {
        flywheel = new SparkFlex(9, MotorType.kBrushless);
        flywheelConfig = new SparkFlexConfig();
        flywheelConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(TurretConstants.flywheelCurrentLimit).closedLoop.pid(TurretConstants.flywheelP.get(), 0.0, TurretConstants.flywheelD.get()).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        flywheelController = flywheel.getClosedLoopController();

        hood = new SparkFlex(10, MotorType.kBrushless);
        hoodConfig = new SparkFlexConfig();
        hoodConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(TurretConstants.hoodCurrentLimit).closedLoop.pid(TurretConstants.hoodP.get(), 0.0, TurretConstants.hoodD.get()).feedbackSensor(FeedbackSensor.kAbsoluteEncoder).outputRange(TurretConstants.hoodMinAngle, TurretConstants.hoodMaxAngle);
        hoodConfig.absoluteEncoder.positionConversionFactor(2*Math.PI);
        hood.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodController = hood.getClosedLoopController();

        turret = new SparkFlex(11, MotorType.kBrushless);
        turretConfig = new SparkFlexConfig();
        turretConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(TurretConstants.turretCurrentLimit).closedLoop.pid(TurretConstants.turretP.get(), 0, TurretConstants.turretD.get()).feedbackSensor(FeedbackSensor.kPrimaryEncoder).outputRange(TurretConstants.turretMinAngle, TurretConstants.turretMaxAngle);
        turretConfig.encoder.positionConversionFactor(2*Math.PI);
        turret.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        turretController = turret.getClosedLoopController();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.flywheelSpeed = turret.getEncoder().getVelocity();
        inputs.hoodAngle = hood.getAbsoluteEncoder().getPosition();
        inputs.turretAngle = turret.getEncoder().getPosition();
        inputs.turretSetAngle = turretSetAngle;

        flywheelController.setSetpoint(flywheelSetSpeed, ControlType.kVelocity);
        hoodController.setSetpoint(hoodSetAngle, ControlType.kPosition);
        turretController.setSetpoint(turretSetAngle, ControlType.kPosition);
    }

    @Override
    public void setFlyWheelSpeed(double speed) {
        flywheelSetSpeed = speed;
    }

    @Override
    public void setHoodAngle(double angle) {
        hoodSetAngle = angle;
    }

    @Override
    public void setTurretAngle(double angle) {
        turretSetAngle = angle;
    }
}
