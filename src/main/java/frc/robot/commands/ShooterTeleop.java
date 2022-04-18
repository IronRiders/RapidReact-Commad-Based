package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.*;
import frc.robot.Constants;

public class ShooterTeleop extends SequentialCommandGroup {
    public ShooterTeleop(ShooterSubsystem shooter, IndexerSubsystem indexer, VisionSubsystem vision, DriveSubsystem drive) {
        addCommands(
            new RunCommand(() -> drive.updateSpeed(0, 0, vision.steeringAssist(), false), drive, vision)
                .alongWith(new InstantCommand(() -> {
                    double minimum = Constants.SHOOTER_MINIMUM_SPEED;
                    double maximum = Constants.SHOOTER_MAXIMUM_SPEED;
                    double aimed = ShooterSubsystem.distanceToRPM(vision.estimateDistance());
                    double rpm = Math.min(Math.max(aimed, minimum), maximum);
                    shooter.shoot(rpm);
                }, shooter, vision))
                .withTimeout(1.5),
            new InstantCommand(drive::stop, drive),
            new InstantCommand(indexer::extend, indexer),
            new WaitCommand(1),
            new InstantCommand(indexer::retract, indexer)
        );
    }
}
