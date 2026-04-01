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
  public static final double intakeD = 1.0e-4;
  public static final double intakekV = 1.811e-3;

  public static final double horizontal1P = 0.0;
  public static final double horizontal1D = 0.0;
  public static final double horizontal1kV = 1.167;

  public static final double horizontal2P = 0.0;
  public static final double horizontal2D = 0.0;
  public static final double horizontal2kV = 1.167;

  public static final double feederP = 1.0e-5;
  public static final double feederD = 0.0;
  public static final double feederkV = 1.811e-3;

  public static final double armMinPosition = 0.0305;
  public static final double armMaxPosition = 0.78;

  public static final double armZeroOffset = 0.31;

  public static final double intakePosition = 0.77;
  public static final double armRestingPosition = 0.15;

  public static final double intakeIntakingSpeed = 5000;
  public static final double feederSpeed = 4500;
  public static final double horizontal1Speed = 9000;
  public static final double horizontal2Speed = 9000;

  public static final double armMaxOutput = 0.25;

  public static final double jamCurrent = 35;
  public static final double jamSpeed = 1000;
  public static final double jamMinCount = 50;
  public static final double unjamMinCount = 10;
}

