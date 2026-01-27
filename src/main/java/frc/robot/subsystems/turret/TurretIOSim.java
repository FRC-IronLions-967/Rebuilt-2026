// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

public class TurretIOSim extends TurretIOSpark {
    public TurretIOSim () {
        super();
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        inputs.flywheelSpeed = flywheelSetSpeed;
        inputs.hoodAngle = hoodSetAngle;
        inputs.turretAngle = turretSetAngle;
        inputs.turretSetAngle = turretSetAngle;
        inputs.resetting = Math.abs(inputs.turretAngle - inputs.turretSetAngle) > Math.PI;
        inputs.intakeSafe = 
            ((TurretConstants.turretIDLEPosition1.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition1.get() + Math.PI/4)
            || ((TurretConstants.turretIDLEPosition2.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition2.get() + Math.PI/4)));
    }
}