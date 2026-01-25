// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class VisionConstants {
  // April Tag Cmaera 1
  public static final String AprilTagCamera1Name = "April_Tag_1";
  public static final Transform3d AprilTagCamera1Transform =
      new Transform3d(
        Units.inchesToMeters(new LoggedNetworkNumber("ATC1x", -11.75).get()),
        Units.inchesToMeters(new LoggedNetworkNumber("ATC1y", 11.75).get()), 
        Units.inchesToMeters(new LoggedNetworkNumber("ATC1z", 9).get()),
        new Rotation3d(0, Units.degreesToRadians(30), Units.degreesToRadians(new LoggedNetworkNumber("ATC1yaw", -135).get())));
  public static final int AprilTagCamera1Index = 0;

  public static final String AprilTagCamera2Name = "April_Tag_2";
  public static final Transform3d AprilTagCamera2Transform =
      new Transform3d(
        Units.inchesToMeters(new LoggedNetworkNumber("ATC2x", -11.75).get()),
        Units.inchesToMeters(new LoggedNetworkNumber("ATC2y", -11.75).get()),
        Units.inchesToMeters(new LoggedNetworkNumber("ATC2z", 9).get()),
        new Rotation3d(0, Units.degreesToRadians(30), new LoggedNetworkNumber("ATC2yaw", 135).get()));
  public static final int AprilTagCamera2Index = 1;

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Tuning constants
  public static final LoggedNetworkNumber maxAmbiguity =
      new LoggedNetworkNumber("Tuning/Vision/maxAmb", 0.3);
  public static final LoggedNetworkNumber maxZError =
      new LoggedNetworkNumber("Tuning/Vision/maxZError", 0.75);

    // public static final int hubAprilTag = DriverStation.getAlliance().get() == Alliance.Red ? 10 : 26;
}
