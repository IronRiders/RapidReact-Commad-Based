package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class MecanumWheel {
    private final CANSparkMax motor;
    private RelativeEncoder encoder;
    private PIDController pidController;
    private static SimpleMotorFeedforward feedForward = new SimpleMotorFeedforward(0, 0, 0);

    public MecanumWheel(int motorId, boolean inverted) {
        motor = new CANSparkMax(motorId, MotorType.kBrushless);
        motor.setInverted(inverted);
        motor.setSmartCurrentLimit(Constants.DRIVE_CURRENT_LIMIT);
        encoder = motor.getEncoder();
        pidController = new PIDController(Constants.AUTO_WHEELPID_KP, 0.0, 0.0);
    }

    public double getVelocity() {
        return (encoder.getVelocity() / 60 * Constants.wheel_circumference);
    }

    public void setVelocity(double mps) {
        motor.setVoltage(feedForward.calculate(mps) + pidController.calculate(getVelocity(), mps));
    }

    public static double getMaxLinearVelocity() {
        return feedForward.maxAchievableVelocity(12.0, 0);
    }
}
