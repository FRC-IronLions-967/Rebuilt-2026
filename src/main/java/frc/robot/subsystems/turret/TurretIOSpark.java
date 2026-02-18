// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import java.util.function.BooleanSupplier;

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

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.BangBangController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.util.LimitSwitchManager;

// import frc.robot.util.LimitSwitchManager;

/** Add your docs here. */
public class TurretIOSpark implements TurretIO{

    protected SparkFlex flywheel;
    protected SparkFlexConfig flywheelConfig;
    // protected SparkClosedLoopController flywheelController;

    protected SparkFlex flywheelFollower;
    protected SparkFlexConfig flywheelFollowerConfig;

    protected BangBangController flywheelBangBang;
    protected SimpleMotorFeedforward flywheelFeedforward;

    protected SparkFlex hood;
    protected SparkFlexConfig hoodConfig;
    protected SparkClosedLoopController hoodController;

    // protected SparkFlex turret;
    // protected SparkFlexConfig turretConfig;
    // protected SparkClosedLoopController turretController;

    protected double flywheelSetSpeed = 0.0;
    protected double hoodSetAngle = 0.35;
    protected double turretSetAngle;

    protected boolean homed;

    // protected BooleanSupplier turretMinLimitSwitch;
    // protected BooleanSupplier turretMaxLimitSwitch;
    // protected boolean lastMaxLimitSwitch;
    // protected boolean lastMinLimitSwitch;

    public TurretIOSpark() {
        flywheel = new SparkFlex(9, MotorType.kBrushless);
        flywheelConfig = new SparkFlexConfig();
        flywheelConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(TurretConstants.flywheelCurrentLimit)
            .inverted(true);//so we have positive values
        flywheel.configure(flywheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // flywheelController = flywheel.getClosedLoopController();

        flywheelFollower = new SparkFlex(10, MotorType.kBrushless);
        flywheelFollowerConfig = new SparkFlexConfig();
        flywheelFollowerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(TurretConstants.flywheelCurrentLimit)
            .follow(flywheel, true);
        flywheelFollower.configure(flywheelFollowerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        flywheelBangBang = new BangBangController();
        flywheelFeedforward = new SimpleMotorFeedforward(TurretConstants.flywheelkS, TurretConstants.flywheelkV, TurretConstants.flywheelkA);

        hood = new SparkFlex(11, MotorType.kBrushless);
        hoodConfig = new SparkFlexConfig();
        // hoodConfig.closedLoop.outputRange(-0.25, 0.25);
        hoodConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(TurretConstants.hoodCurrentLimit)
            .closedLoop
                .pid(TurretConstants.hoodP, 0.0, TurretConstants.hoodD)
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .positionWrappingEnabled(false);
        hoodConfig.encoder.positionConversionFactor(1.0/36.0);
        hood.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        hoodController = hood.getClosedLoopController();

        // turret = new SparkFlex(12, MotorType.kBrushless);
        // turretConfig = new SparkFlexConfig();
        // turretConfig
        //     .idleMode(IdleMode.kBrake)
        //     .smartCurrentLimit(TurretConstants.turretCurrentLimit)
        //     .closedLoop
        //         .pid(TurretConstants.turretP.get(), 0, TurretConstants.turretD.get())
        //         .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        //         .positionWrappingEnabled(false);
        // turretConfig.encoder.positionConversionFactor(2*Math.PI / TurretConstants.turretGearRatio);
        // turretConfig
        //     .softLimit
        //         .forwardSoftLimit(TurretConstants.turretMaxAngle)
        //         .forwardSoftLimitEnabled(true)
        //         .reverseSoftLimit(TurretConstants.turretMinAngle)
        //         .reverseSoftLimitEnabled(true);
        // turret.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        // turretController = turret.getClosedLoopController();

        // turretMinLimitSwitch = LimitSwitchManager.getSwitch(0);
        // turretMaxLimitSwitch = LimitSwitchManager.getSwitch(1);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // if (turretMinLimitSwitch.getAsBoolean() && !lastMinLimitSwitch) {
        //     turret.getEncoder().setPosition(TurretConstants.turretMinAngle);
        // } else if (turretMaxLimitSwitch.getAsBoolean() && !lastMaxLimitSwitch) {
        //     turret.getEncoder().setPosition(TurretConstants.turretMaxAngle);
        // }

        inputs.flywheelSpeed = flywheel.getEncoder().getVelocity();
        inputs.flywheelSetSpeed = flywheelSetSpeed;
        inputs.flywheelCurrent = flywheel.getOutputCurrent()+flywheelFollower.getOutputCurrent();
        inputs.hoodAngle = hood.getAbsoluteEncoder().getPosition();
        inputs.hoodSetAngle = hoodSetAngle;
        inputs.hoodCurrent = hood.getOutputCurrent();
        // inputs.turretAngle = turret.getEncoder().getPosition();
        inputs.turretSetAngle = turretSetAngle;
        // inputs.turretMinLimitSwitch = turretMinLimitSwitch.getAsBoolean();
        // inputs.turretMaxLimitSwitch = turretMaxLimitSwitch.getAsBoolean();
        inputs.resetting = Math.abs(inputs.turretAngle - inputs.turretSetAngle) > Math.PI;
        inputs.intakeSafe = 
            ((TurretConstants.turretIDLEPosition1.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition1.get() + Math.PI/4)
            || ((TurretConstants.turretIDLEPosition2.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition2.get() + Math.PI/4)));

        flywheel.setVoltage(MathUtil.clamp(/*12 * flywheelBangBang.calculate(flywheel.getEncoder().getVelocity(), flywheelSetSpeed) + */flywheelFeedforward.calculate(flywheelSetSpeed), 0, 12));

        // lastMaxLimitSwitch = inputs.turretMaxLimitSwitch;
        // lastMinLimitSwitch = inputs.turretMinLimitSwitch;
    }

    @Override
    public void setFlyWheelSpeed(double speed) {
        flywheelSetSpeed = speed;
    }

    @Override
    public void setHoodAngle(double angle) {
        hoodSetAngle = MathUtil.clamp(angle, TurretConstants.hoodMinAngle, TurretConstants.hoodMaxAngle);
        hoodController.setSetpoint(hoodSetAngle, ControlType.kPosition);
    }

    @Override
    public void setTurretAngle(double angle) {
        angle = MathUtil.angleModulus(angle);

        turretSetAngle = MathUtil.clamp(angle, TurretConstants.turretMinAngle, TurretConstants.turretMaxAngle);
        if (homed) {
            // turretController.setSetpoint(turretSetAngle, ControlType.kPosition);
        }
    }
    
    @Override
    public boolean home() {
        // homed = turretMaxLimitSwitch.getAsBoolean() || turretMinLimitSwitch.getAsBoolean();
        // if (!homed) {
        //     turret.set(TurretConstants.turretHomingSpeed.get());
        // } else {
        //     turret.set(0);
        // }
        // return homed;
        return true;
    }
}
