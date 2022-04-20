package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubSystem;

public class SpitOutBall extends CommandBase {
    private final IntakeSubSystem intakeSubSystem;

    public SpitOutBall(IntakeSubSystem subSystem) {
        intakeSubSystem = subSystem;
        addRequirements(subSystem);
    }

    @Override
    public void initialize() {
        intakeSubSystem.spitOutBall();
    }

    @Override
    public void end(boolean interrupted) {
        intakeSubSystem.stop();
    }
}