// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexExternalEncoderSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;


/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO {

    protected SparkFlex arm;
    private SparkFlexConfig armConfig;
    private SparkClosedLoopController armController;

    protected SparkFlex intakeWheels;
    private SparkFlexConfig intakeWheelConfig;
    private SparkClosedLoopController intakeWheelController;

    public IntakeIOSpark() {
        intakeWheels = new SparkFlex(12, MotorType.kBrushless);
        intakeWheelConfig = new SparkFlexConfig();

        intakeWheelController = intakeWheels.getClosedLoopController();

        intakeWheelConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        intakeWheelConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(1, 0, 0);
        intakeWheels.configure(intakeWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        arm = new SparkFlex(13, MotorType.kBrushless);
        armConfig = new SparkFlexConfig();

        armController = arm.getClosedLoopController();
    }
}
