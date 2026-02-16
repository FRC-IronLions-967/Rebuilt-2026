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

        public double subsystemCurrent;
    }

    /**
     * Updates all sensor readings and current/target values for the intake subsystem.
     * Also manages arm control: if the arm is extended past the threshold and trying to move further out, it stops the motor.
     *
     * @param inputs the IntakeIOInputs object to populate with current sensor values and setpoints
     */
    public default void updateInputs(IntakeIOInputs inputs) {}
 
    /**
     * Sets the desired intake arm angle, clamped to safe limits.
     *
     * @param angle target angle for the intake arm
     */
    public default void setIntakeArmAngle(double angle) {}

    /**
     * Sets the speed of the intake motor.
     *
     * @param speed desired motor speed
     */
    public default void setIntakeSpeed(double speed) {}

    /**
     * Sets the speed of the feeder motor.
     *
     * @param speed desired motor speed
     */
    public default void setFeederSpeed(double speed) {}

    /**
     * Sets the speed of the first horizontal motor.
     *
     * @param speed desired motor speed
     */
    public default void setHorizontal1Speed(double speed) {}    

    /**
     * Sets the speed of the second horizontal motor.
     *
     * @param speed desired motor speed
     */
    public default void setHorizontal2Speed(double speed) {}  
    
    public default void testArm(double speed) {}
}
