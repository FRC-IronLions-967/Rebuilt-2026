// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

/** Add your docs here. */
public class IntakeIOSim extends IntakeIOSpark {
    
    public IntakeIOSim () {}

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.armAngle = armSetAngle;
         inputs.intakeSpeed = intakeSetSpeed;
         inputs.feederSpeed = feederSetSpeed;
         inputs.horizontal1Speed = horizontal1SetSpeed;
         inputs.horizontal2Speed = horizontal2SetSpeed;
    }
}
