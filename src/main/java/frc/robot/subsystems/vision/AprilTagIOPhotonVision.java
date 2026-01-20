// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import java.util.LinkedList;
import java.util.List;

import org.photonvision.PhotonCamera;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

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
          if (result.multitagResult.isPresent()) {
            inputs.poseType = PoseTypes.MULTI;

            var multitagResult = result.getMultiTagResult().get();

            Transform3d fieldToCamera = multitagResult.estimatedPose.best;
            Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
            Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());

            poseObservations.add(new PoseObservation(multitagResult.estimatedPose.ambiguity, robotPose, result.getTimestampSeconds()));
          } else if (!result.targets.isEmpty()) {
            inputs.poseType = PoseTypes.SINGLE;
            var target = result.getTargets().get(0);

            var tagPose = VisionConstants.kTagLayout.getTagPose(target.fiducialId);
            if (tagPose.isPresent()) {
              Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
              Transform3d cameraToTarget = target.bestCameraToTarget;
              Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
              Pose3d robotPose = new Pose3d(fieldToCamera.getTranslation(), fieldToCamera.getRotation());

              poseObservations.add(new PoseObservation(target.poseAmbiguity, robotPose, result.getTimestampSeconds()));
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
        } else {
          inputs.hasTarget = false;
        }
      } 
    }
  }
}
