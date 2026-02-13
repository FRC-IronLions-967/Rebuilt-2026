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
        public boolean turretMinLimitSwitch;
        public boolean turretMaxLimitSwitch;
        public boolean resetting;
        public boolean intakeSafe;

        public double flywheelCurrent;
        public double hoodCurrent;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setFlyWheelSpeed(double speed) {}

    public default void stopFlywheel() {}

    public default void setHoodAngle(double angle) {}

    public default void setTurretAngle(double angle) {}

    public default boolean home() {return false;}
}
