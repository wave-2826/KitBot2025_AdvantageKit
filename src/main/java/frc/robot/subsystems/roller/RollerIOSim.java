package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class RollerIOSim implements RollerIO {
    private DCMotorSim sim = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(DCMotor.getCIM(1), 0.004, motorReduction), DCMotor.getCIM(1));

    private double appliedVolts = 0.0;

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        sim.setInputVoltage(appliedVolts);
        sim.update(0.02);

        inputs.positionRad = sim.getAngularPositionRad();
        inputs.velocityRadPerSec = sim.getAngularVelocityRadPerSec();
        inputs.appliedVolts = appliedVolts;
        inputs.currentAmps = sim.getCurrentDrawAmps();
    }

    @Override
    public void setVoltage(double volts) {
        appliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    }
}
