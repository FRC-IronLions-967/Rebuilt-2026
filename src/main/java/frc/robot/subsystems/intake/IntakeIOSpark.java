// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import java.util.AbstractMap;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.sim.SparkFlexExternalEncoderSim;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;



/** Add your docs here. */
public class IntakeIOSpark implements IntakeIO {
  protected SparkMax intakeHorizontalMotor;
  protected SparkMaxConfig intakeHorizontalMotorConfig;
  protected SparkClosedLoopController intakeHorizontalMotorController;

  protected double horizontalMotorSetSpeed;

  protected SparkFlex feeder;
  protected SparkFlexConfig feederConfig;
  protected SparkClosedLoopController feederController;

  protected double feederSetSpeed;

  protected SparkFlex intake;
  protected SparkFlexConfig intakeConfig;
  protected SparkClosedLoopController intakeController;

  protected double intakeSetSpeed;

  protected SparkFlex arm;
  protected SparkFlexConfig armConfig;
  protected SparkClosedLoopController armController;

  protected double armSetAngle;

  public IntakeIOSpark() {
         intake = new SparkFlex(12, MotorType.kBrushless);
         intakeConfig = new SparkFlexConfig();

         intakeController = intake.getClosedLoopController();

         intakeConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
         intakeConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
         intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

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
         arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
         feeder = new SparkFlex(13, MotorType.kBrushless);
         feederConfig = new SparkFlexConfig();

         feederController = feeder.getClosedLoopController();

         feederConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
         feederConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
         feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

         intakeHorizontalMotor = new SparkMax(13, MotorType.kBrushless);
         intakeHorizontalMotorConfig = new SparkMaxConfig();

         intakeHorizontalMotorController = intakeHorizontalMotor.getClosedLoopController();

         intakeHorizontalMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
         intakeHorizontalMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
         intakeHorizontalMotor.configure(intakeHorizontalMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       }

       @Override
       public void updateInputs(IntakeIOInputs inputs) {
         inputs.armAngle = arm.getAbsoluteEncoder().getPosition();
         inputs.intakeSpeed = intake.getEncoder().getVelocity();
         inputs.feederSpeed = feeder.getEncoder().getVelocity();
         inputs.horizontalMotorSpeed = intakeHorizontalMotor.getEncoder().getVelocity();

         armController.setSetpoint(armSetAngle, ControlType.kPosition);
         intakeController.setSetpoint(intakeSetSpeed, ControlType.kVelocity);
         feederController.setSetpoint(feederSetSpeed, ControlType.kVelocity);
         intakeHorizontalMotorController.setSetpoint(horizontalMotorSetSpeed, ControlType.kVelocity);
       }

       @Override
       public void setIntakeArmAngle(double angle) {
          armSetAngle = angle;
       }

       @Override
       public void setIntakeSpeed(double speed) {
          intakeSetSpeed = speed;
       }

       @Override
       public void setFeederSpeed(double speed) {
          feederSetSpeed = speed;
       }

       @Override
       public void setHorizontalMotorSpeed(double speed) {
          horizontalMotorSetSpeed = speed;
       }

}
