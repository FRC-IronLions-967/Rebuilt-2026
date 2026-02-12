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
        public double armCurrent;

        public double intakeSpeed; 
        public double intakeSetSpeed;
        public double intakeCurrent;

        public double feederSpeed;
        public double feederSetSpeed;
        public double feederCurrent;

        public double horizontal1Speed;
        public double horizontal1SetSpeed;
        public double horizontal1Current;

        public double horizontal2Speed;
        public double horizontal2SetSpeed;
        public double horizontal2Current;
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
 
    public default void setIntakeArmAngle(double angle) {}

    public default void setIntakeSpeed(double speed) {}

    public default void setFeederSpeed(double speed) {}

    public default void setHorizontal1Speed(double speed) {}    

    public default void setHorizontal2Speed(double speed) {}    
}
