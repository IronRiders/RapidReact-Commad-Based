package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.*;

public class TalonSRXCommand extends CommandBase {
    private final TalonSRX motor;
    private final double speed;

    public TalonSRXCommand(Subsystem subsystem, TalonSRX motor, double speed) {
        addRequirements(subsystem);
        this.motor = motor;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        motor.set(TalonSRXControlMode.PercentOutput, speed);
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(TalonSRXControlMode.PercentOutput, 0);
    }
}
