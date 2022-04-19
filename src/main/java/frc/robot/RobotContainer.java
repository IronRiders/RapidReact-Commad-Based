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
                .whenHeld(new TalonSRXCommand(intake, intake.intakeMotor, Constants.INTAKE_SPEED));
        new JoystickButton(controller, 11)
                .whenHeld(new TalonSRXCommand(intake, intake.intakeMotor, -Constants.INTAKE_SPEED));
        // Intake Deployment
        new JoystickButton(controller, 9)
                .whenHeld(new TalonSRXCommand(intake, intake.deploymentMotor, Constants.DEPLOY_SPEED));
        // Climber
        new JoystickButton(controller, 12)
                .whenHeld(new SparkMaxCommand(climber, climber.climber_motor_1, Constants.CLIMBER_POWER))
                .whenHeld(new SparkMaxCommand(climber, climber.climber_motor_2, Constants.CLIMBER_POWER));
        new JoystickButton(controller, 4)
                .whenHeld(new SparkMaxCommand(climber, climber.climber_motor_1, -Constants.CLIMBER_POWER))
                .whenHeld(new SparkMaxCommand(climber, climber.climber_motor_2, -Constants.CLIMBER_POWER));
        // Shoot
        new JoystickButton(controller, 1)
                .whenHeld(new ShooterTeleop(shooter, indexer, vision, drive));
    }

    public Command getAutonomousCommand() {
        return null;
    }
}
