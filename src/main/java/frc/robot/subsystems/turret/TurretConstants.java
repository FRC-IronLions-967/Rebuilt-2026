// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.vision.VisionConstants;
import frc.robot.util.AllianceFlipUtil;

/** Add your docs here. */
public class TurretConstants {

    private static final Translation2d hub = new Translation2d(4.625, 4);
    private static final Translation2d left = new Translation2d(1, 6);
    private static final Translation2d right = new Translation2d(1, 2);
    public static final double allianceZoneEnd = 5;
    public static final double oppositeAllianceEnd = VisionConstants.kTagLayout.getFieldLength() - allianceZoneEnd;
    public static final double fieldEnd = VisionConstants.kTagLayout.getFieldLength();
    public static final double center = 4.0;

    //methods for getting the correct translation based on alliance color: (THIS IS WHY OUR DATA FROM 2/7 WAS BAD)
    public static Translation2d hub() {
        return AllianceFlipUtil.apply(hub);
    }

    public static Translation2d left() {
        return AllianceFlipUtil.apply(left);
    }

    public static Translation2d right() {
        return AllianceFlipUtil.apply(right);
    }

    public static double allianceZoneEnd() {
        return AllianceFlipUtil.applyX(allianceZoneEnd);
    }

    public static double oppositeAllianceEnd() {
        return AllianceFlipUtil.applyX(oppositeAllianceEnd);
    }

    public static double fieldEnd() {
        return AllianceFlipUtil.applyX(fieldEnd);
    }

    public static double center() {
        return AllianceFlipUtil.applyY(center);
    }

    public static final double turretGearRatio = 180.0/44.0;//got from cad 3/18
    public static final double turretOutputRange = 0.5;

    public static final double flywheelP = 1.0e-3;
    public static final double flywheelD = 0.0;

    public static final double turretkS = 0.275;//0.275
    public static final double turretkV = 0.25;
    public static final double flywheelkV = 1.93e-3;
    public static final double flywheelkS = 0.2;

    public static final double hoodP = 5.0;
    public static final double hoodD = 0.0;
    public static final double hoodkS = 1.0;

    public static final LoggedNetworkNumber turretP = new LoggedNetworkNumber("turretP", 1.0);
    public static final LoggedNetworkNumber turretD = new LoggedNetworkNumber("turretD", 0.0);

    public static final LoggedNetworkNumber turretControllerP = new LoggedNetworkNumber("turretControllerP", 5.0e-4);
    public static final LoggedNetworkNumber turretControllerD = new LoggedNetworkNumber("turretControllerD", 0.0);

    public static final int flywheelCurrentLimit = 60;
    public static final int hoodCurrentLimit = 40; 
    public static final int turretCurrentLimit = 30;

    public static final double hoodOffset = -0.028; //used when encoder gets messed up
    public static final double hoodMaxAngle = 0.616 + hoodOffset;
    public static final double hoodMinAngle = 0.239 + hoodOffset;

    public static final double turretMaxAngle = 1.6;
    public static final double turretMinAngle = -4.261;
    public static final double turretStartingAngle = -1.6;
    public static final LoggedNetworkNumber turretHomingSpeed = new LoggedNetworkNumber("Turret Homing Speed", 0.1);

    public static final LoggedNetworkNumber flywheelPassingSpeed = new LoggedNetworkNumber("flywheelPassingSpeed", 2500);
    public static final LoggedNetworkNumber flywheelMinRunningSpeed = new LoggedNetworkNumber("Flywheel Running Min Speed", 1750);
    public static final LoggedNetworkNumber flywheelTolerance = new LoggedNetworkNumber("flywheelTolerance", 1000);
    public static final LoggedNetworkNumber hoodPassingAngle = new LoggedNetworkNumber("Hood Passing Angle", 0.3);

    public static final LoggedNetworkNumber flywheelFullFieldSpeed = new LoggedNetworkNumber("flywheelFullFieldSpeed", 4000);
    public static final LoggedNetworkNumber hoodFullFieldAngle = new LoggedNetworkNumber("Hood FullField Angle", 0.25);

    public static final LoggedNetworkNumber hoodIDLEPosition = new LoggedNetworkNumber("hoodIDLEPosition", hoodMaxAngle);
    public static final double turretIDLEPosition = -1.6;
    public static final double turretTolerance = 0.05;

    public static final ShooterSetpoint startNZ = new ShooterSetpoint(2500, 0.3 + hoodOffset);
    public static final ShooterSetpoint endNZ = new ShooterSetpoint(3250, 0.25 + hoodOffset);


    public static final ShooterSetpoint startFullField = endNZ;
    public static final ShooterSetpoint endFullField = new ShooterSetpoint(4000, 0.25 + hoodOffset);

    public static final LoggedNetworkNumber testingFlywheelSpeed = new LoggedNetworkNumber("Testing Flywheel Speed", 2000);
    public static final LoggedNetworkNumber testingHoodAngle = new LoggedNetworkNumber("Testing Hood Speed", 0.589);

    /*
     * This is how we will tune this number
     * 1) Drive at a constant speed horizontal and shoot a ball.
     * 2) Record where it would have entered
     * 3) find the error in ToF = miss distance / robot speed (miss distance could have a - on it)
     * 4) add the error in ToF to the ToF and divide by the ToF. That is this constant.
     */
    public static final LoggedNetworkNumber ToFRealityConstant = new LoggedNetworkNumber("The Constant of Reality", 1.0);//0.6

    public static final double turretStartingOffset = Math.PI;
    public static final double turretOffsetChange = 0.05;

    public static final LoggedNetworkNumber turretVoltage =new LoggedNetworkNumber("Turret Voltage", 0.0);
}
