package frc.robot.commands;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.*;

public class SparkMaxCommand extends CommandBase {
    private final CANSparkMax motor;
    private final double speed;

    SparkMaxCommand(Subsystem subsystem, CANSparkMax motor, double speed) {
        addRequirements(subsystem);
        this.motor = motor;
        this.speed = speed;
    }

    @Override
    public void initialize() {
        motor.set(speed);
    }

    @Override
    public void end(boolean interrupted) {
        motor.set(0);
    }
}
