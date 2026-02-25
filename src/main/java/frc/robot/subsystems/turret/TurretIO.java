// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        public double flywheelSpeed;
        public double flywheelSetSpeed;
        public double hoodAngle;
        public double hoodSetAngle;
        public double turretAngle;
        public double turretSetAngle;
        public boolean turretLimitSwitch;
        public boolean resetting;
        public boolean intakeSafe;

        public double flywheelCurrent;
        public double hoodCurrent;
    }

    /**
     * Updates sensor readings, control states, and outputs for the turret subsystem.
     *
     * <p>Resets the turret encoder on rising-edge limit switch activation,
     * populates the provided inputs structure with current measurements and
     * setpoints, evaluates safety flags, and applies flywheel voltage using
     * Bang-Bang control with feedforward (clamped to 0–12V).
     *
     * @param inputs Container populated with the latest subsystem data.
     */
    public default void updateInputs(TurretIOInputs inputs) {}

    /**
     * Sets the target flywheel speed in rpm.
     *
     * <p>The commanded speed is stored and applied during the next
     * {@code updateInputs} cycle.
     *
     * @param speed Desired flywheel velocity setpoint.
     */
    public default void setFlyWheelSpeed(double speed) {}

    /**
     * Sets the desired turret angle.
     *
     * <p>The input angle is first normalized to the range (-π, π], then clamped
     * within the configured mechanical limits. The position setpoint is only
     * applied if the turret has been homed.
     *
     * @param angle Desired turret angle (radians).
     */
    public default void setHoodAngle(double angle) {}

    /**
     * Sets the desired hood angle.
     *
     * <p>The requested angle is clamped within the configured mechanical limits
     * before being sent to the hood position controller.
     *
     * @param angle Desired hood angle.
     */
    public default void setTurretAngle(double angle) {}

    /**
     * Attempts to home the turret using limit switches.
     *
     * <p>If neither limit switch is triggered, the turret moves at the homing speed.
     * Once a switch is triggered, the turret starts going to {@code turretSetAngle}.
     *
     * @return true if the turret is homed, false otherwise.
     */
    public default boolean home() {return false;}

    public default void testTurret(double speed) {}

    public default void stopTurret() {}
}
