package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants; 

public class ClimberSubsystem extends SubsystemBase {
    public CANSparkMax climber_motor_1;
    public CANSparkMax climber_motor_2;

    public ClimberSubsystem() {
        int currentLimit = 10;

        climber_motor_1 = new CANSparkMax(Constants.CLIMBER_PORT_1, MotorType.kBrushless);
        climber_motor_1.setIdleMode(IdleMode.kBrake);
        climber_motor_1.setSmartCurrentLimit(currentLimit);

        climber_motor_2 = new CANSparkMax(Constants.CLIMBER_PORT_2, MotorType.kBrushless);
        climber_motor_2.setIdleMode(IdleMode.kBrake);
        climber_motor_2.setSmartCurrentLimit(currentLimit);
    }
}
