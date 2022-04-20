package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubSystem;

public class IntakeBallCommand extends CommandBase {
    private final IntakeSubSystem intakeSubSystem;

    public IntakeBallCommand(IntakeSubSystem subSystem) {
        intakeSubSystem = subSystem;
        addRequirements(subSystem);
    }

    @Override
    public void initialize() {
        intakeSubSystem.intakeBall();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubSystem.stop();
    }
}