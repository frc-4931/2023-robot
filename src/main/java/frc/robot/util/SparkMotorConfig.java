package frc.robot.util;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;

public class SparkMotorConfig {
  
  public static CANSparkMax motor(int canId) {
    return motor(canId, IdleMode.kCoast, false);
  }

  public static CANSparkMax motor(int canId, IdleMode idleMode, boolean inverted) {
    CANSparkMax motor = new CANSparkMax(canId, MotorType.kBrushless);
    motor.enableVoltageCompensation(12d);
    motor.setInverted(inverted);
    motor.setIdleMode(idleMode);
    return motor;
  }

  public static SparkMaxPIDController pidController(CANSparkMax motor, double kP, double kI, double kD,
      double kFF, double maxVelocity, double maxAcceleration, 
      double outputRangeLow, double outputRangeHigh, double allowedClosedLoopError) {
    SparkMaxPIDController pidController = motor.getPIDController();
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setFF(kFF);
    pidController.setOutputRange(outputRangeLow, outputRangeHigh);
    // pidController.setFeedbackDevice(null)
    // pidController.setPositionPIDWrappingEnabled(false);
    // pidController.setPositionPIDWrappingMaxInput();
    // pidController.setPositionPIDWrappingMinInput();
    pidController.setSmartMotionAllowedClosedLoopError(allowedClosedLoopError, 0);
    pidController.setSmartMotionMaxAccel(maxAcceleration, 0);
    pidController.setSmartMotionMaxVelocity(maxVelocity, 0);

    return pidController;
  }

  public static AbsoluteEncoder absoluteEncoder(CANSparkMax motor, boolean inverted, boolean zero) {
    AbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    encoder.setInverted(inverted);

    // encoder.setZeroOffset(encoder.getPosition());
    motor.getPIDController().setFeedbackDevice(encoder);

    return encoder;
  }

  // public static RelativeEncoder relativeEncoder(CANSparkMax motor, boolean inverted) {
  //   RelativeEncoder encoder = motor.

  // }
}
