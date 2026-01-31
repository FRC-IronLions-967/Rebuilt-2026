// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.vision.AprilTagIO.TargetInfo;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;

  private final VisionConsumer consumer;

  private boolean rejectPose;

  /** Creates a new AprilTagVision. */
  public AprilTagVision(VisionConsumer consumer, AprilTagIO... io) {
    this.io = io;
    this.consumer = consumer;
    this.inputs = new AprilTagIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagIOInputsAutoLogged();
    }
  }

  /**
   * @param cameraIndex what camera to look
   * @param aprilTagID what april tag to look at
   * @return the rot and norm of the specified april tag
   */
  public TargetInfo getTargetInfo(int cameraIndex, int aprilTagID) {
    for (int i = 0; i < inputs[cameraIndex].targetInfo.length; i++) {
      if (inputs[cameraIndex].targetInfo[i].tagID() == aprilTagID) {
        return inputs[cameraIndex].targetInfo[i];
      }
    }
    return null;
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/AprilTag/Camera" + Integer.toString(i), inputs[i]);
    }
    // Update Pose
    for (int cameraIndex = 0; cameraIndex < inputs.length; cameraIndex++) {
      for (var obs : inputs[cameraIndex].poseObservations) {
        // filtering
        rejectPose =
            obs.ambiguity() > VisionConstants.maxAmbiguity.get()
                || obs.pose().getZ() > VisionConstants.maxZError.get()
                || obs.pose().getX() < 0
                || obs.pose().getY() < 0
                || obs.pose().getX() > VisionConstants.kTagLayout.getFieldLength()
                || obs.pose().getY() > VisionConstants.kTagLayout.getFieldWidth();

        if (rejectPose) continue;

        double stdFactor = Math.pow(obs.avgTagDistance(), 2) / obs.tagCount();
        double linearStdDev = VisionConstants.linearStdDevBaseline * stdFactor;
        double angularStdDev = VisionConstants.angularStdDevBaseline * stdFactor;


        consumer.accept(obs.pose().toPose2d(), obs.timestamp(), VecBuilder.fill(linearStdDev, linearStdDev, angularStdDev));
      }
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
      Pose2d pose,
      double timestamp,
      Matrix<N3, N1> visionMeasurementStdDevs);
  }
}
