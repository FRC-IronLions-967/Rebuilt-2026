// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

/** Add your docs here. */
public interface TurretIO {
    @AutoLog
    public static class TurretIOInputs {
        double flywheelSpeed;
        double hoodAngle;
        double turretAngle;
        double turretSetAngle;
        boolean turretMinLimitSwitch;
        boolean turretMaxLimitSwitch;
    }

    public default void updateInputs(TurretIOInputs inputs) {}

    public default void setFlyWheelSpeed(double speed) {}

    public default void setHoodAngle(double angle) {}

    public default void setTurretAngle(double angle) {}

    public default void setFieldRelativeTurretAngle(Rotation2d wantedRotation, Rotation2d robotRotation) {}
}
