// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeConstants{

  public static final double armP = 1.0;
  public static final double armD = 0.0;
  public static final double armkS = 2.0;

  public static final double intakeP = 1.0e-3;
  public static final double intakeD = 0.0;
  public static final double intakekV = 1.811e-3;

  public static final double armMinPosition = 0.0305;
  public static final double armMaxPosition = 0.698;

  public static final double armZeroOffset = 0.3353;

  public static final double intakePosition = 0.698;
  public static final double armRestingPosition = 0.05;

  public static final double intakeIntakingSpeed = 5000;
  public static final double feederSpeed = 0.75;
  public static final double horizontal1Speed = 0.85;
  public static final double horizontal2Speed = 0.85;

  public static final double armMaxOutput = 0.25;

  public static final double jamCurrent = 35;
  public static final double jamSpeed = 1000;
  public static final double jamMinCount = 50;
  public static final double unjamMinCount = 10;
}

