package frc.robot.commands;
import javax.naming.ldap.StartTlsResponse;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;

public class CommandFactory {
    public static SequentialCommandGroup runIndexerCommand(IndexerSubsystem indexer) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> indexer.extend(), indexer),
                new WaitCommand(1),
                new InstantCommand(() -> indexer.retract(), indexer));
    }

    // 5 Ball Auto Path Untested (Don't mess with them)
    public SequentialCommandGroup FiveBallAuto(ShooterSubsystem shooter, DriveSubsystem drive, IntakeSubSystem intaker, IndexerSubsystem indexer, VisionSubsystem vision) {
        return new SequentialCommandGroup(

        new InstantCommand(() -> drive.resetOdometry(new Pose2d(
            Trajectory.getInitialPose().getTranslation(), 
            ((PathPlannerState)((PathPlannerTrajectory)trajectory).sample(0)).holonomicRotation),

        new ParallelDeadlineGroup("FiveBall1", drive),
        new RunCommand(intaker::intakeBall, intaker)),
        new WaitCommand(1),
        new ShooterTeleop(shooter, indexer, vision, drive),

        new ParallelDeadlineGroup("FiveBall2", drive),
        new RunCommand(intaker::intakeBall, intaker),
        new WaitCommand(1),

        new ParallelDeadlineGroup("FiveBall3", drive),
        new ShooterTeleop(shooter, indexer, vision, drive),

        new ParallelDeadlineGroup("FiveBall4", drive),
        new RunCommand(intaker::intakeBall, intaker),
        new WaitCommand(1),

        new ParallelDeadlineGroup("FiveBall5", drive),
        new ShooterTeleop(shooter, indexer, vision, drive));
    }
}