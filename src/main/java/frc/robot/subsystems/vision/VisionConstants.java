// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

/** Add your docs here. */
public class VisionConstants {
  // April Tag Cmaera 1
  public static final String AprilTagCameraName = "April_Tag";
  public static final Transform3d AprilTagCameraTransform =
      new Transform3d(
          Units.inchesToMeters(6.376), 0, Units.inchesToMeters(8.85), new Rotation3d(0, 0, 0));
  public static final int AprilTagCameraIndex = 0;

  // Object Detection Camera
  public static final String ObjectDetectionCameraName = "ObjectDetectionCamera";
  public static final Transform3d ObjectDetectionCameraTransform = new Transform3d();
  public static final int ObjectDetectionCameraIndex = 1;

  public static final AprilTagFieldLayout kTagLayout =
      AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

  // Tuning constants
  public static final LoggedNetworkNumber maxAmbiguity =
      new LoggedNetworkNumber("Tuning/Vision/maxAmb", 0.3);
  public static final LoggedNetworkNumber maxZError =
      new LoggedNetworkNumber("Tuning/Vision/maxZError", 0.75);

    public static final int hubAprilTag = DriverStation.getAlliance().get() == Alliance.Red ? 10 : 26;
}
