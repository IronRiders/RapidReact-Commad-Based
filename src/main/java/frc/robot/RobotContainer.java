// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.ShooterTeleop;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    public final VisionSubsystem vision = new VisionSubsystem();
    public final ShooterSubsystem shooter = new ShooterSubsystem();
    public final DriveSubsystem drive = new DriveSubsystem();
    public final IndexerSubsystem indexer = new IndexerSubsystem();
    public final IntakeSubSystem intake = new IntakeSubSystem();
    public final ClimberSubsystem climber = new ClimberSubsystem();

    public static GenericHID controller = new GenericHID(0);

    public RobotContainer() {
        // Drive
        drive.setDefaultCommand(new RunCommand(
                () -> drive.updateSpeed(controller.getRawAxis(0), controller.getRawAxis(1),
                        controller.getRawAxis(3),
                        true), drive));
        // Invert Drive
        new JoystickButton(controller, 3).whenPressed(new InstantCommand(drive::invertDrive, drive));
        // Intake
        new JoystickButton(controller, 2)
            .whenPressed(new InstantCommand(intake::intakeBall, intake))
            .whenReleased(new InstantCommand(intake::stop, intake));
        new JoystickButton(controller, 11)
            .whenHeld(new InstantCommand(intake::spitOutBall, intake))
            .whenReleased(new InstantCommand(intake::stop, intake));
        // Intake Deployment
        new JoystickButton(controller, 9).whenPressed(new InstantCommand(intake::startDeployment, intake));
        new JoystickButton(controller, 10).whenPressed(new InstantCommand(intake::finishDeployment, intake));
        // Climber
        new JoystickButton(controller, 12)
            .whenHeld(new InstantCommand(climber::raise, climber))
            .whenReleased(new InstantCommand(climber::stop, climber));
        // Lower Climber
        new JoystickButton(controller, 4)
            .whenHeld(new InstantCommand(climber::lower, climber))
            .whenReleased(new InstantCommand(climber::stop, climber));
        // Shoot
        // new JoystickButton(controller, 1)
        //     .whenHeld(new ShooterTeleop(shooter, indexer, vision, drive));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
