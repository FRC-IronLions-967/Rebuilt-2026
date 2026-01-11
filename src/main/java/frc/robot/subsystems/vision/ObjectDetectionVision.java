// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class ObjectDetectionVision extends SubsystemBase {

  private final ObjectDetectionIO[] io;
  private final ObjectDetectionIOInputsAutoLogged[] inputs;

  /** Creates a new Vision. */
  public ObjectDetectionVision(ObjectDetectionIO... io) {
    this.io = io;

    this.inputs = new ObjectDetectionIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new ObjectDetectionIOInputsAutoLogged();
    }
  }

  /**
   * @param cameraIndex which camera to use
   * @return distance to best object
   */
  public double getDistanceToObject(int cameraIndex) {
    return inputs[cameraIndex].distanceToTarget;
  }

  /**
   * @param cameraIndex which camera to use
   * @return a Rotation2d with yaw value
   */
  public Rotation2d getObjectX(int cameraIndex) {
    return inputs[cameraIndex].xRot;
  }

  /**
   * idk why you would use this
   *
   * @param cameraIndex which camera to use
   * @return a Rotation2d with pitch value
   */
  public Rotation2d getObjectY(int cameraIndex) {
    return inputs[cameraIndex].yRot;
  }

  /**
   * only used if nonsymetic game piece
   *
   * @param cameraIndex which camera to use
   * @return a Rotation2d with yaw value
   */
  public Rotation2d getObjectZ(int cameraIndex) {
    return inputs[cameraIndex].zRot;
  }

  /**
   * @param cameraIndex which camera to use
   * @return if the camera has an object
   */
  public boolean hasTarget(int cameraIndex) {
    return inputs[cameraIndex].hasTarget;
  }

  @Override
  public void periodic() {
    // Update Inputs
    for (int i = 0; i < inputs.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/ObjectDetection/Camera" + Integer.toString(i), inputs[i]);
    }
  }
}
