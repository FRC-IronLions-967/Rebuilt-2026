// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeConstants{

  public static final double armP = 1.0;
  public static final double armD = 0.0;
  public static final double armkS = 2.0;

  public static final double intakeP = 0.0;
  public static final double intakeD = 0.0;
  public static final double intakekV = 1.0;

  public static final double armMinPosition = 0.173;//0.176
  public static final double armMaxPosition = 0.86;
  public static final double armOutThreshold = 0.48;

  public static final double armZeroOffset = 0.5;

  public static final double intakePosition = 0.85;
  public static final double armRestingPosition = 0.3;

  public static final double intakeIntakingSpeed = 3000;
  public static final double feederSpeed = 0.75;
  public static final double horizontal1Speed = 0.85;
  public static final double horizontal2Speed = 0.65;

  public static final double armMaxOutput = 0.25;

  public static final double jamCurrent = 35;
  public static final double jamSpeed = 1000;
  public static final double jamMinCount = 50;
  public static final double unjamMinCount = 50;
}

