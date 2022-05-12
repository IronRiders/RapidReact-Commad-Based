package frc.robot.commands;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.MecanumWheel;

public class MecanumPathFollower extends PPMecanumControllerCommand {
    private static ProfiledPIDController thetaController = new ProfiledPIDController(Constants.AUTO_THETACONTROLLER_KP, 0, 0, 
    new TrapezoidProfile.Constraints(Units.rotationsToRadians(0.75), Units.rotationsToRadians(1.5)));
    private static PIDController xController = new PIDController(Constants.AUTO_POSITION_KP, 0, 0);
    private static PIDController yController = new PIDController(Constants.AUTO_POSITION_KP, 0, 0);

    public MecanumPathFollower(PathPlannerTrajectory trajectory, DriveSubsystem drive) {
       super(trajectory, 
       () -> drive.getPose2d(), 
       drive.getKinematics(), 
       xController, yController, thetaController, 
       MecanumWheel.getMaxLinearVelocity(), (speed) -> drive.SetWheelSpeeds(speed), drive);
    }
}
