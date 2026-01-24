// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexExternalEncoderSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
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

        intakeWheelConfig
                .idleMode(IdleMode.kCoast)
                .smartCurrentLimit(40);
        intakeWheelConfig
                .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
        intakeWheels
            .configure(
                    intakeWheelConfig, 
                    ResetMode.kResetSafeParameters, 
                    PersistMode.kNoPersistParameters);
        arm = new SparkFlex(13, MotorType.kBrushless);
        armConfig = new SparkFlexConfig();

        armController = arm.getClosedLoopController();

        armConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
        armConfig
                .absoluteEncoder
                .apply(new AbsoluteEncoderConfig())
                .positionConversionFactor(2 * Math.PI)
                .zeroOffset(IntakeConstants.armZeroOffset);
        armConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                .pid(IntakeConstants.armP, IntakeConstants.armI, IntakeConstants.armD)
                .outputRange(-IntakeConstants.armPercentPower, IntakeConstants.armPercentPower);
        
        intakeWheelConfig
                .closedLoop
                .feedbackSensor(
                    FeedbackSensor.kPrimaryEncoder)
                    .pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
        arm.configure(
            armConfig, 
            ResetMode.kResetSafeParameters, 
            PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeArmAngle = arm.getAbsoluteEncoder().getPosition();
        inputs.intakeWheelSpeed = intakeWheels.getEncoder().getVelocity();
    }

    @Override
  public void runIntakeWheels(double speed) {
    intakeWheelController.setReference(speed, ControlType.kVelocity);
  }

  @Override
  public void moveIntakeArm(double angle) {
    armController.setReference(angle, ControlType.kPosition);
  }
}
