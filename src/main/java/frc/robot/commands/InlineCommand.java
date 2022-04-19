package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;

public class InlineCommand extends CommandBase {
    private final Runnable init;
    private final Runnable end;

    public InlineCommand(Runnable init, Runnable end, Subsystem... requirements) {
        addRequirements(requirements);
        this.init = init;
        this.end = end;
    }

    @Override
    public void initialize() {
        init.run();
    }

    @Override
    public void end(boolean interrupted) {
        end.run();
    }
}
