package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KA;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KG;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KS;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KV;
import static frc.robot.Constants.ArmConstants.LIFT_MOTOR;
import static frc.robot.Constants.ArmConstants.WINCH_MOTOR;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  private ArmFeedforward armFeedforward;
  private CANSparkMax liftMotor, winchMotor;
  private SparkMaxPIDController liftPidController;
  private RelativeEncoder liftEncoder;
  private double goalPosition;

  public Arm() {
    armFeedforward = new ArmFeedforward(FEED_FORWARD_KS, FEED_FORWARD_KG, FEED_FORWARD_KV, FEED_FORWARD_KA);
    liftMotor = LIFT_MOTOR.createMotor();
    liftPidController = liftMotor.getPIDController();
    liftEncoder = liftMotor.getEncoder();

    goalPosition = Units.degreesToRadians(90);
    
    winchMotor = WINCH_MOTOR.createMotor();
  }

  @Override
  public void periodic() {
    // get the current angle, velocity & acceleration
    var angle = liftEncoder.getPosition();
    var velocity = liftEncoder.getVelocity();
    var acceleration = 0; // does not seem possible to know

    // calculate the FF
    var ff = armFeedforward.calculate(angle, velocity, acceleration);

    // set the reference point
    // pidController.setReference(goalPosition, ControlType.kSmartMotion, 0, ff, ArbFFUnits.kVoltage);

    // calc max length
    
    SmartDashboard.putNumber("Arm position", angle);
    SmartDashboard.putNumber("Arm Alt Encoder Velocity", velocity);
    SmartDashboard.putNumber("Arm Applied Output", liftMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm calculated FF", ff);
    // SmartDashboard.putData("Arm Lift Motor", liftMotor);
    
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
    liftMotor.set(value * .5);
  }

  public void runWinch(double value) {
    winchMotor.set(value * .5);
  }

  public Command stop() {
    return this.runOnce(() -> {
      winchMotor.set(0);
      liftMotor.set(0);
    });
  }
}
