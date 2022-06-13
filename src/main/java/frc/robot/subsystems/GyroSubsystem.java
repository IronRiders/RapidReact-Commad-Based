package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.kauailabs.navx.frc.AHRS;

public class GyroSubsystem {
    private AHRS gyro;

    public GyroSubsystem() {
        gyro = new AHRS(SerialPort.Port.kMXP);
    }

    public double getAngle() {
        return gyro.getAngle();
    }
}
