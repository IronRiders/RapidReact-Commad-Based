package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTeleop extends CommandBase {
    private ShooterSubsystem m_shooter;
    private IndexerSubsystem m_indexer;
    private DriveSubsystem m_drive;

    public ShooterTeleop(ShooterSubsystem shooter, IndexerSubsystem indexer, DriveSubsystem drive) {
        m_shooter = shooter;
        m_indexer = indexer;
        m_drive = drive;
        addRequirements(m_shooter);
        addRequirements(m_drive);
        addRequirements(m_indexer);
    }

    @Override
    public void initialize() {
        m_indexer.retract();
    }

    @Override
    public void execute() {
            new RunCommand(m_drive::updateSpeedAuto, m_drive)
            .alongWith(new RunCommand(m_shooter::shootAuto, m_shooter))
            .withTimeout(1.5)
            .andThen(m_drive::stop, m_drive)
            .andThen(m_indexer::extend, m_indexer)
            .andThen(new WaitCommand(1));
    }

    public void end() {
        m_shooter.stop();
        m_indexer.retract();
    }
}
