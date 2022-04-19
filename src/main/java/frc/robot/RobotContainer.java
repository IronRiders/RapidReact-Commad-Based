package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

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
        drive.setDefaultCommand(new RunCommand(() -> drive.updateSpeed(controller.getRawAxis(0),
                controller.getRawAxis(1), controller.getRawAxis(2), true)));
        // Invert Drive
        new JoystickButton(controller, 3).whenPressed(new InstantCommand(drive::invertDrive, drive));
        // Intake
        new JoystickButton(controller, 2)
                .whenHeld(new StartEndCommand(intake::intakeBall, intake::stop, intake));        
        new JoystickButton(controller, 11)
                .whenHeld(new StartEndCommand(intake::spitOutBall, intake::stop, intake));
        // Intake Deployment
        new JoystickButton(controller, 9)
                .whenHeld(new StartEndCommand(intake::startDeployment, intake::finishDeployment, intake));
        // Climber
        new JoystickButton(controller, 12)
                .whenHeld(new StartEndCommand(climber::raise, climber::stop, climber));
        new JoystickButton(controller, 4)
                .whenHeld(new StartEndCommand(climber::lower, climber::stop, climber));
        // Shoot
        new JoystickButton(controller, 1)
                .whenHeld(new ShooterTeleop(shooter, indexer, vision, drive));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
