// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

/** Add your docs here. */
public interface ObjectDetectionIO {
  @AutoLog
  public static class ObjectDetectionIOInputs {
    public boolean isConnected = false;
    public boolean hasTarget = false;
    public Rotation2d xRot = new Rotation2d();
    public Rotation2d yRot = new Rotation2d();
    public Rotation2d zRot = new Rotation2d();
    public double distanceToTarget;
  }

  public default void updateInputs(ObjectDetectionIOInputs inputs) {}
}
