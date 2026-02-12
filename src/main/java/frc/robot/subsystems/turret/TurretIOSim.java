// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.networktables.LoggedNetworkNumber;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.LinearSystemSim;
// import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Robot;

public class TurretIOSim extends TurretIOSpark {

    // private final FlywheelSim flywheelSim;
    private SingleJointedArmSim turretSim;

    private final PIDController turretSimPID =
        new PIDController(TurretConstants.turretP, 0.0, TurretConstants.turretD);

    private SimpleMotorFeedforward turretSimFeeedforward = new SimpleMotorFeedforward(TurretConstants.turretkS, 0.0);
    private LoggedNetworkNumber volts = new LoggedNetworkNumber("volts", 0.0);
    
    public TurretIOSim () {
        super();
        // flywheelSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(2), 0.0009, 0.7), DCMotor.getNeoVortex(2));
        turretSim = new SingleJointedArmSim(
            DCMotor.getNeoVortex(1), 
            1/TurretConstants.turretGearRatio, 
            0.022, 
            .2, 
            TurretConstants.turretMinAngle, 
            TurretConstants.turretMaxAngle, 
            false, 
            0.0
        );
    }

    @Override
    public void updateInputs(TurretIOInputs inputs) {
        // inputs.flywheelSpeed = flywheelSim.getAngularVelocityRPM();
        // flywheelSim.setInputVoltage(12 * flywheelBangBang.calculate(flywheelSim.getAngularVelocityRPM(), flywheelSetSpeed) + flywheelFeedforward.calculate(flywheelSetSpeed));

        // flywheelSim.update(Robot.defaultPeriodSecs);

        // inputs.flywheelSetSpeed = flywheelSetSpeed;
        // inputs.hoodAngle = hoodSetAngle;
        inputs.turretAngle = turretSim.getAngleRads();
        inputs.turretSetAngle = turretSetAngle;

        turretSim.setInputVoltage(MathUtil.clamp(turretSimPID.calculate(inputs.turretAngle, turretSetAngle) + turretSimFeeedforward.calculate(turretSetAngle), -12, 12));
        // turretSim.setInputVoltage(volts.get());
        turretSim.update(Robot.defaultPeriodSecs);

        inputs.resetting = Math.abs(inputs.turretAngle - inputs.turretSetAngle) > Math.PI;
        inputs.intakeSafe = 
            ((TurretConstants.turretIDLEPosition1.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition1.get() + Math.PI/4)
            || ((TurretConstants.turretIDLEPosition2.get() - Math.PI/4 < inputs.turretAngle) && (inputs.turretAngle < TurretConstants.turretIDLEPosition2.get() + Math.PI/4)));
    }
}