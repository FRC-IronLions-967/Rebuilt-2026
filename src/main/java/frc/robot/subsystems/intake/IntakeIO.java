// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double armAngle;
        public double armSetAngle;

        public double intakeSpeed; 
        public double intakeSetSpeed;

        public double feederSpeed;
        public double feederSetSpeed;

        public double horizontalMotor1Speed;
        public double horizontalMotor1SetSpeed;

        public double horizontalMotor2Speed;
        public double horizontalMotor2SetSpeed;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
 
    public default void setIntakeArmAngle(double angle) {}

    public default void setIntakeSpeed(double speed) {}

    public default void setFeederSpeed(double speed) {}

    public default void setHorizontalMotor1Speed(double speed) {}    

    public default void setHorizontalMotor2Speed(double speed) {}    
}
