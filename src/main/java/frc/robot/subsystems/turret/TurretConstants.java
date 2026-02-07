// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class TurretConstants {
    public static final double turretHeight = 1.0;
    public static final Translation2d hub = new Translation2d(4.625, 4);
    public static final Translation2d left = new Translation2d(1, 7);
    public static final Translation2d right = new Translation2d(1, 1);
    public static final Translation2d[] trenches = {new Translation2d(4.66, .66), new Translation2d(4.66, 7.33), new Translation2d(11.925, 0.66), new Translation2d(11.925, 7.33)};
    public static final double trenchTolerance = 1.0;

    public static final double turretGearRatio = 1.0;

    public static final double flywheelP = 1.0e-3;
    public static final double flywheelD = 1.0e-6;

    public static final LoggedNetworkNumber hoodP = new LoggedNetworkNumber("hoodP", 1);
    public static final LoggedNetworkNumber hoodD = new LoggedNetworkNumber("hoodD", 0.0);

    public static final LoggedNetworkNumber turretP = new LoggedNetworkNumber("turretP", 1.0);
    public static final LoggedNetworkNumber turretD = new LoggedNetworkNumber("turretD", 0.0);

    public static final LoggedNetworkNumber turretControllerP = new LoggedNetworkNumber("turretControllerP", 1.0e-5);
    public static final LoggedNetworkNumber turretControllerD = new LoggedNetworkNumber("turretControllerD", 0.0);

    public static final int flywheelCurrentLimit = 40;
    public static final int hoodCurrentLimit = 40; 
    public static final int turretCurrentLimit = 30;

    public static final double hoodMaxAngle = 0.9;
    public static final double hoodMinAngle = 0.5;

    public static final double turretMaxAngle = Math.PI-0.0872;
    public static final double turretMinAngle = -Math.PI+0.0872;
    public static final double turretTolerance = 0.0872; //5 degrees

    public static final LoggedNetworkNumber flywheelPassingSpeed = new LoggedNetworkNumber("flywheelPassingSpeed", -6784);
    public static final LoggedNetworkNumber flywheelShootingSpeed = new LoggedNetworkNumber("flywheelShootingSpeed", -4000);
    public static final LoggedNetworkNumber flywheelTolerance = new LoggedNetworkNumber("flywheelTolerance", 250);

    public static final LoggedNetworkNumber hoodIDLEPosition = new LoggedNetworkNumber("hoodIDLEPosition", hoodMaxAngle);
    public static final LoggedNetworkNumber turretIDLEPosition1 = new LoggedNetworkNumber("turretIDLEPosition1", Math.PI/2);
    public static final LoggedNetworkNumber turretIDLEPosition2 = new LoggedNetworkNumber("turretIDLEPosition2", -Math.PI/2);
}
