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

   protected SparkMax horizontal1;
   protected SparkMaxConfig horizontal1Config;

   protected double horizontal1SetSpeed;

   protected SparkMax horizontal2;
   protected SparkMaxConfig horizontal2Config;

   protected double horizontal2SetSpeed;

   protected SparkFlex feeder;
   protected SparkFlexConfig feederConfig;

   protected double feederSetSpeed;

   protected SparkFlex intake;
   protected SparkFlexConfig intakeConfig;

   protected double intakeSetSpeed;

   protected SparkFlex arm;
   protected SparkFlexConfig armConfig;
   protected SparkClosedLoopController armController;

   protected double armSetAngle = IntakeConstants.intakePosition;

   public IntakeIOSpark() {
      intake = new SparkFlex(13, MotorType.kBrushless);
      intakeConfig = new SparkFlexConfig();

      intakeConfig
         .idleMode(IdleMode.kCoast)
         .smartCurrentLimit(40);
      intake.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      arm = new SparkFlex(14, MotorType.kBrushless);
      armConfig = new SparkFlexConfig();
      armController = arm.getClosedLoopController();

      armConfig
         .idleMode(IdleMode.kBrake);
      armConfig
            .absoluteEncoder
            .zeroOffset(IntakeConstants.armZeroOffset);
      armConfig
            .closedLoop
            .outputRange(-IntakeConstants.armMaxOutput, IntakeConstants.armMaxOutput)
            .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
            .pid(IntakeConstants.armP, 0.0, IntakeConstants.armD);
            // .feedForward
               // .kS(IntakeConstants.armkS);
      arm.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   
      feeder = new SparkFlex(15, MotorType.kBrushless);
      feederConfig = new SparkFlexConfig();

      feederConfig
         .idleMode(IdleMode.kCoast)
         .smartCurrentLimit(40)
         .inverted(true);
      feeder.configure(feederConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      horizontal1 = new SparkMax(16, MotorType.kBrushless);
      horizontal1Config = new SparkMaxConfig();

      horizontal1Config
         .idleMode(IdleMode.kCoast)
         .smartCurrentLimit(40);
      horizontal1.configure(horizontal1Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

      horizontal2 = new SparkMax(17, MotorType.kBrushless);
      horizontal2Config = new SparkMaxConfig();

      horizontal2Config
         .idleMode(IdleMode.kCoast)
         .smartCurrentLimit(40)
         .inverted(true);
      horizontal2.configure(horizontal2Config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
   }

   @Override
   public void updateInputs(IntakeIOInputs inputs) {
      inputs.armAngle = arm.getAbsoluteEncoder().getPosition();
      inputs.intakeSpeed = intake.getEncoder().getVelocity();
      inputs.feederSpeed = feeder.getEncoder().getVelocity();
      inputs.horizontal1Speed = horizontal1.getEncoder().getVelocity();
      inputs.horizontal2Speed = horizontal2.getEncoder().getVelocity();

      inputs.armSetAngle = armSetAngle;
      inputs.intakeSetSpeed = intakeSetSpeed;
      inputs.feederSetSpeed = feederSetSpeed;
      inputs.horizontal1SetSpeed = horizontal1SetSpeed;
      inputs.horizontal2SetSpeed = horizontal2SetSpeed;

      inputs.armCurrent = arm.getOutputCurrent();
      inputs.feederCurrent = feeder.getOutputCurrent();
      inputs.intakeCurrent = intake.getOutputCurrent();
      inputs.horizontal1Current = horizontal1.getOutputCurrent();
      inputs.horizontal2Current = horizontal2.getOutputCurrent();

      inputs.subsystemCurrent = inputs.armCurrent + inputs.intakeCurrent+inputs.feederCurrent+inputs.horizontal1Current+inputs.horizontal2Current;

      armController.setSetpoint(armSetAngle, ControlType.kPosition);
   }

   @Override
   public void setIntakeArmAngle(double angle) {
      armSetAngle = MathUtil.clamp(angle, IntakeConstants.armMinPosition, IntakeConstants.armMaxPosition);
   }

   @Override
   public void setIntakeSpeed(double speed) {
      intakeSetSpeed = speed;
      // intake.set(intakeSetSpeed);
   }

   @Override
   public void setFeederSpeed(double speed) {
      feederSetSpeed = speed;
      // feeder.set(feederSetSpeed);
   }

   @Override
   public void setHorizontal1Speed(double speed) {
      horizontal1SetSpeed = speed;
      // horizontal1.set(horizontal1SetSpeed);
   }

   @Override
   public void setHorizontal2Speed(double speed) {
      horizontal2SetSpeed = speed;
      // horizontal2.set(horizontal2SetSpeed);
   }

   @Override
   public void testArm(double speed) {
      //  arm.setVoltage(speed);
   }
}
//jamming next (delete after lunch)