// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;

public class IntakeIOSpark implements IntakeIO {

   protected SparkMax horizontalMotor1;
   protected SparkMaxConfig horizontalMotor1Config;
   protected SparkClosedLoopController horizontalMotor1Controller;

   protected double horizontalMotor1SetSpeed;

   protected SparkMax horizontalMotor2;
   protected SparkMaxConfig horizontalMotor2Config;
   protected SparkClosedLoopController horizontalMotor2Controller;

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
      intake = new SparkFlex(13, MotorType.kBrushless);
      intakeConfig = new SparkFlexConfig();
      intakeController = intake.getClosedLoopController();

      intakeConfig
         .idleMode(IdleMode.kCoast)
         .smartCurrentLimit(40)
         .inverted(false)
         .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.intakeP, IntakeConstants.intakeI, IntakeConstants.intakeD);
      intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      arm = new SparkFlex(14, MotorType.kBrushless);
      armConfig = new SparkFlexConfig();

      armConfig
         .idleMode(IdleMode.kBrake)
         .smartCurrentLimit(40)
         .absoluteEncoder
            .zeroOffset(IntakeConstants.armZeroOffset)
            .inverted(false);
      armConfig
         .closedLoop
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(IntakeConstants.armP, IntakeConstants.armI, IntakeConstants.armD);
      arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
      armController = arm.getClosedLoopController();
      
      feeder = new SparkFlex(15, MotorType.kBrushless);
      feederConfig = new SparkFlexConfig();
      feederController = feeder.getClosedLoopController();

      feederConfig
         .idleMode(IdleMode.kBrake)
         .smartCurrentLimit(40)
         .inverted(false)
         .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.feederP, IntakeConstants.feederI, IntakeConstants.feederD);
      feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      horizontalMotor1 = new SparkMax(16, MotorType.kBrushless);
      horizontalMotor1Config = new SparkMaxConfig();
      horizontalMotor1Controller = horizontalMotor1.getClosedLoopController();

      horizontalMotor1Config
         .idleMode(IdleMode.kCoast)
         .smartCurrentLimit(40)
         .inverted(false)
         .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.horizontalMotor1P, IntakeConstants.horizontalMotor1I, IntakeConstants.horizontalMotor1D);
      horizontalMotor1.configure(horizontalMotor2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      horizontalMotor2 = new SparkMax(17, MotorType.kBrushless);
      horizontalMotor2Config = new SparkMaxConfig();
      horizontalMotor2Controller = horizontalMotor2.getClosedLoopController();

      horizontalMotor2Config
         .idleMode(IdleMode.kCoast)
         .smartCurrentLimit(40)
         .inverted(false)
         .closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .pid(IntakeConstants.horizontalMotor2P, IntakeConstants.horizontalMotor2I, IntakeConstants.horizontalMotor2D);
      horizontalMotor2.configure(horizontalMotor1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   @Override
   public void updateInputs(IntakeIOInputs inputs) {
      inputs.armAngle = arm.getAbsoluteEncoder().getPosition();
      inputs.armSetAngle = armSetAngle;
      inputs.intakeSpeed = intake.getEncoder().getVelocity();
      inputs.intakeSetSpeed = intakeSetSpeed;
      inputs.feederSpeed = feeder.getEncoder().getVelocity();
      inputs.feederSetSpeed = feederSetSpeed;
      inputs.horizontalMotor1Speed = horizontalMotor1.getEncoder().getVelocity();
      inputs.horizontalMotor1SetSpeed = horizontalMotor1SetSpeed;
      inputs.horizontalMotor2Speed = horizontalMotor2.getEncoder().getVelocity();
      inputs.horizontalMotor2SetSpeed = horizontalMotor2SetSpeed;
   }

   @Override
   public void setIntakeArmAngle(double angle) {
      armSetAngle = MathUtil.clamp(angle, IntakeConstants.armMinPosition, IntakeConstants.armMaxPosition);
      armController.setSetpoint(armSetAngle, ControlType.kPosition);
   }

   @Override
   public void setIntakeSpeed(double speed) {
      intakeSetSpeed = speed;
      intakeController.setSetpoint(intakeSetSpeed, ControlType.kVelocity);
   }

   @Override
   public void setFeederSpeed(double speed) {
      feederSetSpeed = speed;
      feederController.setSetpoint(feederSetSpeed, ControlType.kVelocity);
   }

   @Override
   public void setHorizontalMotor1Speed(double speed) {
      horizontalMotor1SetSpeed = speed;
      horizontalMotor1Controller.setSetpoint(horizontalMotor1SetSpeed, ControlType.kVelocity);
   }

   @Override
   public void setHorizontalMotor2Speed(double speed) {
      horizontalMotor2SetSpeed = speed;
      horizontalMotor2Controller.setSetpoint(horizontalMotor2SetSpeed, ControlType.kVelocity);
   }
}
