// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

public class TurretIOSim extends TurretIOSpark {

    private final FlywheelSim flywheelSim;
    
    public TurretIOSim () {
        super();
        flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), 0.5, 2), DCMotor.getNeoVortex(2), 1);
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        flywheelSim.update(Robot.defaultPeriodSecs);

        inputs.flywheelSpeed = flywheelSim.getAngularVelocityRPM();
        inputs.flywheelSetSpeed = flywheelSetSpeed;
        inputs.hoodAngle = hoodSetAngle;
        // inputs.turretAngle = turretSetAngle;
        // inputs.turretSetAngle = turretSetAngle;
        inputs.resetting = Math.abs(inputs.turretAngle - inputs.turretSetAngle) > Math.PI;
        inputs.intakeSafe = 
            ((TurretConstants.turretIDLEPosition1.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition1.get() + Math.PI/4)
            || ((TurretConstants.turretIDLEPosition2.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition2.get() + Math.PI/4)));
    }
}