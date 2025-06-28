package frc.robot.subsystems.roller;

import static frc.robot.subsystems.roller.RollerConstants.*;
import static frc.robot.util.PhoenixUtil.tryUntilOkV5;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

/** This drive implementation is for a Talon SRX driving a brushed motor. */
public class RollerIOTalonSRX implements RollerIO {
    private final TalonSRX roller = new TalonSRX(rollerCanId);

    public RollerIOTalonSRX() {
        var config = new TalonSRXConfiguration();
        config.peakCurrentLimit = currentLimit;
        config.continuousCurrentLimit = currentLimit - 15;
        config.peakCurrentDuration = 250;
        config.voltageCompSaturation = 12.0;

        tryUntilOkV5(5, () -> roller.configAllSettings(config));
    }

    @Override
    public void updateInputs(RollerIOInputs inputs) {
        inputs.appliedVolts = roller.getMotorOutputVoltage();
        inputs.currentAmps = roller.getStatorCurrent();
    }

    @Override
    public void setVoltage(double volts) {
        roller.set(TalonSRXControlMode.PercentOutput, volts / 12.0);
    }
}
