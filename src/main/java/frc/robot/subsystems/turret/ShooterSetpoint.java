// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.interpolation.Interpolatable;

/** Add your docs here. */
public class ShooterSetpoint implements Interpolatable<ShooterSetpoint>{

    public double rpm;
    public double hoodAngle;

    public ShooterSetpoint(double rpm, double hoodAngle) {
        this.rpm = rpm;
        this.hoodAngle = hoodAngle;
    }

    @Override
    public ShooterSetpoint interpolate(ShooterSetpoint endValue, double t) {
        return new ShooterSetpoint(
            rpm + (endValue.rpm-rpm) * t,
            hoodAngle + (endValue.hoodAngle-hoodAngle) * t
        );
    }
}
