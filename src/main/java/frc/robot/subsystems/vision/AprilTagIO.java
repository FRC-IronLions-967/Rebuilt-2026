// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;

import java.util.function.Supplier;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;

/** Add your docs here. */
public interface AprilTagIO {
  @AutoLog
  public static class AprilTagIOInputs implements LoggableInputs {
    public boolean isConnected = false;
    public boolean hasTarget = false;
    public TargetInfo[] targetInfo;
    public PoseObservation[] poseObservations = new PoseObservation[0];
    @Override
    public void toLog(LogTable table) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'toLog'");
    }
    @Override
    public void fromLog(LogTable table) {
      // TODO Auto-generated method stub
      throw new UnsupportedOperationException("Unimplemented method 'fromLog'");
    }
  }

  public static record TargetInfo(int tagID, double targetYaw, double targetPitch, double distanceToTarget) {}

  public static record PoseObservation(
      double ambiguity, Pose3d pose, boolean hasTags, double timestamp) {}

  public static record VisionPoseObs(Pose2d poseObs, boolean poseObsGood, double timestamp) {}

  public default void updateInputs(AprilTagIOInputs inputs) {}

  public default void givePoseSupplier(Supplier<Pose2d> poseSupplier) {} // Only use for sim
}
