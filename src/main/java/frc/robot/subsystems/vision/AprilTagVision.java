// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.vision.AprilTagIO.PoseObservation;
import frc.robot.subsystems.vision.AprilTagIO.TargetInfo;
import frc.robot.subsystems.vision.AprilTagIO.VisionPoseObs;
import java.util.LinkedList;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class AprilTagVision extends SubsystemBase {

  private final AprilTagIO[] io;
  private final AprilTagIOInputsAutoLogged[] inputs;
  private Pose2d acceptedPose;
  private boolean acceptedPoseGood;
  private double timestamp;

  // Periodic variables HERE TO NOT RUN OUT OF MEM
  private List<PoseObservation> robotObservatons = new LinkedList<>();
  private List<PoseObservation> robotObservatonsAccepted = new LinkedList<>();
  private List<PoseObservation> robotObservatonsRejected = new LinkedList<>();
  private boolean rejectPose;

  /** Creates a new AprilTagVision. */
  public AprilTagVision(AprilTagIO... io) {
    this.io = io;

    this.inputs = new AprilTagIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new AprilTagIOInputsAutoLogged();
    }

    acceptedPose = new Pose2d();
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

  public VisionPoseObs getPoseObs() {
    return new VisionPoseObs(acceptedPose, acceptedPoseGood, timestamp);
  }

  /**
   * Only to be used with sim to fix looping inits
   *
   * @param poseSupplier posesupplier for sim
   */
  public void setPoseSupplierIfSim(Supplier<Pose2d> poseSupplier) {
    if (Constants.currentMode == Constants.Mode.SIM) {
      for (int i = 0; i < io.length; i++) {
        io[i].givePoseSupplier(poseSupplier);
      }
    }
  }

  @Override
  public void periodic() {
    // set values to reset
    robotObservatons.clear();
    robotObservatonsRejected.clear();
    robotObservatonsAccepted.clear();

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
                || !obs.hasTags()
                || obs.pose().getX() < 0
                || obs.pose().getY() < 0
                || obs.pose().getX() > VisionConstants.kTagLayout.getFieldLength()
                || obs.pose().getY() > VisionConstants.kTagLayout.getFieldWidth();

        robotObservatons.add(obs);
        if (rejectPose) {
          robotObservatonsRejected.add(obs);
        } else {
          robotObservatonsAccepted.add(obs);
        }
      }
    }

    /*Returns an accepted pose
     * Uses the pose with the best ambiguity
     */
    if (robotObservatonsAccepted.size() >= 1) {
      int minAmb = 10000;
      for (int i = 0; i < robotObservatonsAccepted.size(); i++) {
        if (robotObservatonsAccepted.get(i).ambiguity() < minAmb) {
          acceptedPose = robotObservatonsAccepted.get(i).pose().toPose2d();
          timestamp = robotObservatonsAccepted.get(i).timestamp();
        }
      }
      acceptedPoseGood = true;
    } else {
      acceptedPoseGood = false;
    }

    Logger.recordOutput("Vision/AprilTag/AcceptedPoseGood", acceptedPoseGood);
    Logger.recordOutput("Vision/AprilTag/TotalVisionMesurmentsCount", robotObservatons.size());
    Logger.recordOutput("Vision/AprilTag/RejectedPoseCount", robotObservatonsRejected.size());
    Logger.recordOutput(
        "Vision/AprilTag/AcceptedVisionMesurmentsCount", robotObservatonsAccepted.size());
  }
}
