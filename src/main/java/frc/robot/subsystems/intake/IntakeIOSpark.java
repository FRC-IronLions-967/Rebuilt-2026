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
  protected SparkMax intakeHorizontalMotor1;
  protected SparkMaxConfig intakeHorizontalMotor1Config;
  protected SparkClosedLoopController intakeHorizontalMotor1Controller;

  protected double horizontalMotor1SetSpeed;

  protected SparkMax intakeHorizontalMotor2;
  protected SparkMaxConfig intakeHorizontalMotor2Config;
  protected SparkClosedLoopController intakeHorizontalMotor2Controller;

  protected double horizontalMotor2SetSpeed;

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

         intakeHorizontalMotor1 = new SparkMax(13, MotorType.kBrushless);
         intakeHorizontalMotor1Config = new SparkMaxConfig();

         intakeHorizontalMotor1Controller = intakeHorizontalMotor1.getClosedLoopController();

         intakeHorizontalMotor1Config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
         intakeHorizontalMotor1Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
         intakeHorizontalMotor1.configure(intakeHorizontalMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

         intakeHorizontalMotor2 = new SparkMax(14, MotorType.kBrushless);
         intakeHorizontalMotor2Config = new SparkMaxConfig();

         intakeHorizontalMotor2Controller = intakeHorizontalMotor2.getClosedLoopController();

         intakeHorizontalMotor2Config.idleMode(IdleMode.kBrake).smartCurrentLimit(40);
         intakeHorizontalMotor2Config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
         intakeHorizontalMotor2.configure(intakeHorizontalMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
       }

       @Override
       public void updateInputs(IntakeIOInputs inputs) {
         inputs.armAngle = arm.getAbsoluteEncoder().getPosition();
         inputs.intakeSpeed = intake.getEncoder().getVelocity();
         inputs.feederSpeed = feeder.getEncoder().getVelocity();
         inputs.horizontalMotor1Speed = intakeHorizontalMotor1.getEncoder().getVelocity();
         inputs.horizontalMotor2Speed = intakeHorizontalMotor2.getEncoder().getVelocity();

         armController.setSetpoint(armSetAngle, ControlType.kPosition);
         intakeController.setSetpoint(intakeSetSpeed, ControlType.kVelocity);
         feederController.setSetpoint(feederSetSpeed, ControlType.kVelocity);
         intakeHorizontalMotor1Controller.setSetpoint(horizontalMotor1SetSpeed, ControlType.kVelocity);
         intakeHorizontalMotor2Controller.setSetpoint(horizontalMotor1SetSpeed, ControlType.kVelocity);
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
       public void setHorizontalMotor1Speed(double speed) {
          horizontalMotor1SetSpeed = speed;
       }

       @Override
       public void setHorizontalMotor2Speed(double speed) {
          horizontalMotor2SetSpeed = speed;
       }
}
