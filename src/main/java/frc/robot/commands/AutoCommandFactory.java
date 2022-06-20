package frc.robot.commands;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPlannerTrajectory.PathPlannerState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubSystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AutoCommandFactory {

  public SequentialCommandGroup FiveBallAuto(ShooterSubsystem shooter, DriveSubsystem drive, IntakeSubSystem intaker,
      IndexerSubsystem indexer, VisionSubsystem vision) {

    return new SequentialCommandGroup(
        new ParallelDeadlineGroup(
            autoPath("FiveBall1", drive, true),
            new RunCommand(intaker::intakeBall, intaker)),
        new ShooterTeleop(shooter, indexer, vision, drive),

        new ParallelDeadlineGroup(
            autoPath("FiveBall2", drive, false),
            new RunCommand(intaker::intakeBall, intaker)),

        autoPath("FiveBall3", drive, false),
        new ShooterTeleop(shooter, indexer, vision, drive),

        new ParallelDeadlineGroup(
            autoPath("FiveBall4", drive, false),
            new RunCommand(intaker::intakeBall, intaker)),

        autoPath("FiveBall5", drive, false),
        new ShooterTeleop(shooter, indexer, vision, drive));
  }

  public Command autoPath(String p, DriveSubsystem drive, boolean initial) {
    PathPlannerTrajectory path = PathPlanner.loadPath(p, Constants.DRIVE_SPEED_AUTO, Constants.DRIVE_ACCELERATION_AUTO);
    Command cmd = new MecanumPathFollower(path, drive);
    if (initial) {
      cmd = cmd.beforeStarting(() -> drive.resetOdometry(new Pose2d(
          path.getInitialPose().getTranslation(),
          ((PathPlannerState) path.sample(0)).holonomicRotation)));
    }
    return cmd;
  }
}
