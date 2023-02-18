package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KA;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KG;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KS;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KV;
import static frc.robot.Constants.ArmConstants.ARM_MOTOR;
import static frc.robot.Constants.ArmConstants.WINCH_MOTOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private static final double MAX_EXTENSION = Units.inchesToMeters(48 + 12.5);
  private static final double MAX_HEIGHT = Units.inchesToMeters(56);
  private static final double BASE_LENGTH = Units.inchesToMeters(0); // TODO: 
  private static final double NINETY_DEG = Math.PI / 2;
  private static final double THETA_MAX = Units.degreesToRadians(42.78);

  private ArmFeedforward armFeedforward;
  private CANSparkMax armMotor, winchMotor;
  private SparkMaxPIDController armPidController;
  private RelativeEncoder armEncoder;
  private double armPosition;
  private double winchPosition;

  public Arm() {
    armFeedforward = new ArmFeedforward(FEED_FORWARD_KS, FEED_FORWARD_KG, FEED_FORWARD_KV, FEED_FORWARD_KA);
    armMotor = ARM_MOTOR.createMotor();
    armPidController = armMotor.getPIDController();
    armEncoder = armMotor.getEncoder();

    armPosition = Units.degreesToRadians(90);
        
    winchMotor = WINCH_MOTOR.createMotor();
    winchPosition = 0;
  }

  @Override
  public void periodic() {
    // get the current angle, velocity & acceleration
    var theta = armEncoder.getPosition();
    var velocity = armEncoder.getVelocity();
    var acceleration = 0; // does not seem possible to know

    // calculate the FF
    var ff = armFeedforward.calculate(theta, velocity, acceleration);

    // set the reference point
    armPidController.setReference(armPosition, ControlType.kSmartMotion, 0, ff, ArbFFUnits.kVoltage);

    // calc max length
    var thetaMod = MathUtil.inputModulus(theta, 0, NINETY_DEG);
    var maxLength = thetaMod < THETA_MAX ? MAX_EXTENSION / Math.cos(thetaMod) : MAX_HEIGHT / Math.sin(thetaMod);

    var len = maxLength - BASE_LENGTH;
    winchMotor.setSoftLimit(SoftLimitDirection.kForward, (float) len);
    winchMotor.getPIDController().setReference(Math.max(winchPosition, len), ControlType.kSmartMotion);
    
    SmartDashboard.putNumber("Arm position", theta);
    SmartDashboard.putNumber("Arm Alt Encoder Velocity", velocity);
    SmartDashboard.putNumber("Arm Applied Output", armMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm calculated FF", ff);
    SmartDashboard.putNumber("Arm Max Length", maxLength);
    
    SmartDashboard.putNumber("Winch Position", winchMotor.getEncoder().getPosition());
    SmartDashboard.putNumber("Winch Velocity", winchMotor.getEncoder().getVelocity());
    SmartDashboard.putNumber("Winch Applied Output", winchMotor.getAppliedOutput());
    

  }

  private void setGoal(double position) {

  }
  
  public Command setPositionCommand() {
    return runOnce(() -> setGoal(0));
  }

  public void runArm(double value) {
    armMotor.set(value * .5);
  }

  public void runWinch(double value) {
    winchMotor.set(value * .5);
  }

  public Command stop() {
    return this.runOnce(() -> {
      winchMotor.set(0);
      armMotor.set(0);
    });
  }
}
