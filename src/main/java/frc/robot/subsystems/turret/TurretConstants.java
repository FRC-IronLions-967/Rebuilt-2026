// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.vision.VisionConstants;

/** Add your docs here. */
public class TurretConstants {

    public static Translation2d flipForRed(Translation2d bluePose) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return new Translation2d(
                VisionConstants.kTagLayout.getFieldLength() - bluePose.getX(),
                VisionConstants.kTagLayout.getFieldWidth()  - bluePose.getY()
            );
        }
        return bluePose;
    }

    private static final Translation2d hub = new Translation2d(4.625, 4);
    private static final Translation2d left = new Translation2d(1, 7);
    private static final Translation2d right = new Translation2d(1, 1);

    //methods for getting the correct translation based on alliance color: (THIS IS WHY OUR DATA FROM 2/7 WAS BAD)
    public static Translation2d hub() {
        return flipForRed(hub);
    }

    public static Translation2d left() {
        return flipForRed(left);
    }

    public static Translation2d right() {
        return flipForRed(right);
    }

    public static final Translation2d[] trenches = {new Translation2d(4.66, .66), new Translation2d(4.66, 7.33), new Translation2d(11.925, 0.66), new Translation2d(11.925, 7.33)};
    public static final double trenchTolerance = 1.0;

    public static final double turretGearRatio = 1.0;

    public static final double flywheelP = 1.0e-3;
    public static final double flywheelD = 1.0e-6;

    public static final double flywheelkS = 0.0;
    public static final double flywheelkV = 1.89e-3;
    public static final double flywheelkA = 0.0;

    public static final double hoodP = 1.0;
    public static final double hoodD = 0.0;
    public static final double hoodkS = 1.0;

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
    public static final LoggedNetworkNumber turretHomingSpeed = new LoggedNetworkNumber("Turret Homing Speed", 0.1);

    public static final LoggedNetworkNumber flywheelPassingSpeed = new LoggedNetworkNumber("flywheelPassingSpeed", 1500);
    public static final LoggedNetworkNumber flywheelShootingSpeed = new LoggedNetworkNumber("flywheelShootingSpeed", 1500);//speed not velocity for bang bang
    public static final LoggedNetworkNumber flywheelTolerance = new LoggedNetworkNumber("flywheelTolerance", 250);
    public static final LoggedNetworkNumber hoodPassingAngle = new LoggedNetworkNumber("Hood Passing Angle", 0.9);

    public static final LoggedNetworkNumber hoodIDLEPosition = new LoggedNetworkNumber("hoodIDLEPosition", hoodMaxAngle);
    public static final LoggedNetworkNumber turretIDLEPosition1 = new LoggedNetworkNumber("turretIDLEPosition1", Math.PI/2);
    public static final LoggedNetworkNumber turretIDLEPosition2 = new LoggedNetworkNumber("turretIDLEPosition2", -Math.PI/2);
}
