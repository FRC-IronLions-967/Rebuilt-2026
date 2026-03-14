// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Robot;

/** Add your docs here. */
public class IntakeIOSim extends IntakeIOSpark {

    private final FlywheelSim intakeSim;
    
    public IntakeIOSim () {
        super();
        intakeSim = new FlywheelSim(LinearSystemId.createFlywheelSystem(DCMotor.getNeoVortex(1), 0.0009, 0.7), DCMotor.getNeoVortex(1));
    }

    @Override
    public void updateInputs(IntakeIOInputs inputs) {
        inputs.intakeSpeed = intakeSim.getAngularVelocityRPM();
        intakeSim.setInputVoltage(MathUtil.clamp(intakFeedforward.calculate(intakeSetSpeed), -12, 12));

        intakeSim.update(Robot.defaultPeriodSecs);

        inputs.intakeSetSpeed = intakeSetSpeed;

        inputs.armAngle = armSetAngle;
        inputs.feederSpeed = feederSetSpeed;
        inputs.horizontal1Speed = horizontal1SetSpeed;
        inputs.horizontal2Speed = horizontal2SetSpeed;
    }
}
