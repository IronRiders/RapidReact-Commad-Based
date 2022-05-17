package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

public class DriveSubsystem extends SubsystemBase {
    private boolean inverted;
    private CANSparkMax[] motors;

    public MecanumDrive drive;

    public  DriveSubsystem() {
        this.motors = new CANSparkMax[4];
        this.motors[0] = new CANSparkMax(Constants.WHEEL_PORT_FRONT_LEFT, MotorType.kBrushless);
        this.motors[1] = new CANSparkMax(Constants.WHEEL_PORT_FRONT_RIGHT, MotorType.kBrushless);
        this.motors[2] = new CANSparkMax(Constants.WHEEL_PORT_REAR_LEFT, MotorType.kBrushless);
        this.motors[3] = new CANSparkMax(Constants.WHEEL_PORT_REAR_RIGHT, MotorType.kBrushless);

        this.motors[0].setInverted(true);
        this.motors[1].setInverted(false);
        this.motors[2].setInverted(true);
        this.motors[3].setInverted(false);
        inverted = false;

        motors[0].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        motors[1].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        motors[2].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        motors[3].setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);

        drive = new MecanumDrive(this.motors[0], this.motors[1], this.motors[2], this.motors[3]);
    }

    public void invertDrive() {
        inverted = !inverted;
    }
}
