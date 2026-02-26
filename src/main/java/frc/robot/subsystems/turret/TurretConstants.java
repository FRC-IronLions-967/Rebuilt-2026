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
    private static final Translation2d left = new Translation2d(1, 6);
    private static final Translation2d right = new Translation2d(1, 2);
    public static final double allianceZoneEnd = 5;
    public static final double oppositeAllianceEnd = VisionConstants.kTagLayout.getFieldLength() - allianceZoneEnd;
    public static final double fieldEnd = VisionConstants.kTagLayout.getFieldLength();
    public static final double center = 4.0;

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

    public static double flipXLineForRed(double line) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return VisionConstants.kTagLayout.getFieldLength() - line;
        }
        return line;
    }

    public static double flipYLineForRed(double line) {
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            return VisionConstants.kTagLayout.getFieldWidth() - line;
        }
        return line;
    }

    public static double allianceZoneEnd() {
        return flipXLineForRed(allianceZoneEnd);
    }

    public static double oppositeAllianceEnd() {
        return flipXLineForRed(oppositeAllianceEnd);
    }

    public static double fieldEnd() {
        return flipXLineForRed(fieldEnd);
    }

    public static double center() {
        return flipYLineForRed(center);
    }

    public static final double turretGearRatio = 46.02;//got from cad 2/16
    public static final double turretOutputRange = 0.5;

    public static final double flywheelP = 1.0e-3;
    public static final double flywheelD = 1.0e-6;

    public static final double flywheelkS = 0.0;
    public static final double flywheelkV = 1.84e-3;
    public static final double flywheelkA = 0.0;

    public static final double hoodP = 5.0;
    public static final double hoodD = 0.0;
    public static final double hoodkS = 1.0;

    public static final LoggedNetworkNumber turretP = new LoggedNetworkNumber("turretP", 1.0);
    public static final LoggedNetworkNumber turretD = new LoggedNetworkNumber("turretD", 0.0);

    public static final LoggedNetworkNumber turretControllerP = new LoggedNetworkNumber("turretControllerP", 1.0e-5);
    public static final LoggedNetworkNumber turretControllerD = new LoggedNetworkNumber("turretControllerD", 0.0);

    public static final int flywheelCurrentLimit = 50;
    public static final int hoodCurrentLimit = 40; 
    public static final int turretCurrentLimit = 30;

    public static final double hoodOffset = -0.012; //used when encoder gets messed up
    public static final double hoodMaxAngle = 0.616 + hoodOffset;
    public static final double hoodMinAngle = 0.239 + hoodOffset;

    public static final double turretMaxAngle = 1.6;
    public static final double turretMinAngle = -2.60;
    public static final double turretStartingAngle = -1.6;
    public static final LoggedNetworkNumber turretHomingSpeed = new LoggedNetworkNumber("Turret Homing Speed", 0.1);

    public static final LoggedNetworkNumber flywheelPassingSpeed = new LoggedNetworkNumber("flywheelPassingSpeed", 2500);
    public static final LoggedNetworkNumber flywheelMinRunningSpeed = new LoggedNetworkNumber("Flywheel Running Min Speed", 1000);
    public static final LoggedNetworkNumber flywheelTolerance = new LoggedNetworkNumber("flywheelTolerance", 250);
    public static final LoggedNetworkNumber hoodPassingAngle = new LoggedNetworkNumber("Hood Passing Angle", 0.3 + hoodOffset);

    public static final LoggedNetworkNumber flywheelFullFieldSpeed = new LoggedNetworkNumber("flywheelFullFieldSpeed", 4000);
    public static final LoggedNetworkNumber hoodFullFieldAngle = new LoggedNetworkNumber("Hood FullField Angle", 0.25);

    public static final LoggedNetworkNumber hoodIDLEPosition = new LoggedNetworkNumber("hoodIDLEPosition", hoodMaxAngle);
    public static final double turretIDLEPosition = -1.6;
    public static final double turretTolerance = 0.1;

    public static final ShooterSetpoint startNZ = new ShooterSetpoint(2500, 0.3 + hoodOffset);
    public static final ShooterSetpoint endNZ = new ShooterSetpoint(3250, 0.25 + hoodOffset);


    public static final ShooterSetpoint startFullField = endNZ;
    public static final ShooterSetpoint endFullField = new ShooterSetpoint(4000, 0.25 + hoodOffset);

    public static final LoggedNetworkNumber testingFlywheelSpeed = new LoggedNetworkNumber("Testing Flywheel Speed", 2500);
    public static final LoggedNetworkNumber testingHoodAngle = new LoggedNetworkNumber("Testing Flywheel Speed", 0.616 + hoodOffset);
}
