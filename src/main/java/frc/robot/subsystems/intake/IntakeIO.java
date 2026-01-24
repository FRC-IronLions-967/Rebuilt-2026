// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface IntakeIO {
    @AutoLog
    public static class IntakeIOInputs{
        public double intakeArmAngle;
        public double intakeWheelSpeed;        
    }

    public default void updateInputs(IntakeIOInputs inputs) {}
 
    public default void moveIntakeArm(double angle) {}

    public default void runIntakeWheels(double speed) {}
}
