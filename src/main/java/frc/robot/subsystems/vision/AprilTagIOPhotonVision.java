// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonUtils;

/** Add your docs here. */
public class AprilTagIOPhotonVision implements AprilTagIO {

  protected PhotonCamera camera;
  protected Transform3d robotToCamera = null;

  public AprilTagIOPhotonVision(String cameraName, Transform3d robotToCamera) {
    camera = new PhotonCamera(cameraName);
    this.robotToCamera = robotToCamera;
  } 

  @Override
  public void updateInputs(AprilTagIOInputs inputs) {
    inputs.isConnected = camera.isConnected();
    if (inputs.isConnected) {
      List<PoseObservation> poseObservations = new LinkedList<>();
      List<TargetInfo> targetInfos = new LinkedList<>();
      for (var result : camera.getAllUnreadResults()) {
        inputs.hasTarget = result.hasTargets();
        // update inputs
        if (inputs.hasTarget) {
          targetInfos.clear();
          for (var target : result.targets) {
            targetInfos.add(
                new TargetInfo(target.getFiducialId(), target.getYaw(), target.getPitch(), target.getBestCameraToTarget().getTranslation().getNorm()));
          }
          // update pose
          if (VisionConstants.kTagLayout
              .getTagPose(result.getBestTarget().getFiducialId())
              .isPresent() && robotToCamera != null) {
            poseObservations.add(
                new PoseObservation(
                    result.getBestTarget().getPoseAmbiguity(),
                    PhotonUtils.estimateFieldToRobotAprilTag(
                        result.getBestTarget().getBestCameraToTarget(),
                        VisionConstants.kTagLayout
                            .getTagPose(result.getBestTarget().getFiducialId())
                            .get(),
                        robotToCamera),
                    inputs.hasTarget,
                    result.getTimestampSeconds()));
          }
        }
        inputs.targetInfo = new TargetInfo[targetInfos.size()];
        for (int i = 0; i < targetInfos.size(); i++) {
          inputs.targetInfo[i] = targetInfos.get(i);
        }
        inputs.poseObservations = new PoseObservation[poseObservations.size()];
        for (int i = 0; i < poseObservations.size(); i++) {
          inputs.poseObservations[i] = poseObservations.get(i);
        }
      }
    } else {
      inputs.hasTarget = false;
    }
  }
}
