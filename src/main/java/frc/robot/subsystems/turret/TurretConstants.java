// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class TurretConstants {
    public static final LoggedNetworkNumber flywheelP = new LoggedNetworkNumber("flywheelP", 1.0);
    public static final LoggedNetworkNumber flywheelD = new LoggedNetworkNumber("flywheelD", 0.0);

    public static final LoggedNetworkNumber hoodP = new LoggedNetworkNumber("hoodP", 1.0);
    public static final LoggedNetworkNumber hoodD = new LoggedNetworkNumber("hoodD", 0.0);

    public static final LoggedNetworkNumber turretP = new LoggedNetworkNumber("turretP", 1.0);
    public static final LoggedNetworkNumber turretD = new LoggedNetworkNumber("turretD", 0.0);

    public static final int flywheelCurrentLimit = 40;
    public static final int hoodCurrentLimit = 20; 
    public static final int turretCurrentLimit = 30;

    public static final double hoodMaxAngle = Math.PI/2;
    public static final double hoodMinAngle = Math.PI/4;

    public static final double turretMaxAngle = 1.5 * Math.PI;
    public static final double turretMinAngle = -1.5 * Math.PI;

    public static final LoggedNetworkNumber flywheelPassingSpeed = new LoggedNetworkNumber("flywheelPassingSpeed", 6784);
    public static final LoggedNetworkNumber flywheelShootingSpeed = new LoggedNetworkNumber("flywheelShootingSpeed", 6784);

    public static final LoggedNetworkNumber hoodIDLEPosition = new LoggedNetworkNumber("hoodIDLEPosition", hoodMaxAngle);

    public static final PIDController turretAngleController =
        new PIDController(
            new LoggedNetworkNumber("turretControllerP", 1.0).get(),
            0.0,
            new LoggedNetworkNumber("turretControllerD", 0.0).get());
}
