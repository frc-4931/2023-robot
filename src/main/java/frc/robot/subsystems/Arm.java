package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.ARM_MOTOR;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KA;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KG;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KS;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KV;
import static frc.robot.Constants.ArmConstants.WINCH_MOTOR;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  public enum ArmHeight {
    FLOOR(0), CONE_1(0), CUBE_1(0), CONE_2(0), CUBE_2(0), LOADING(0), TRAVELING(0);

    double position;
    ArmHeight(double p) {
      position = p;
    }
  }
  private static final double MAX_EXTENSION = Units.inchesToMeters(48 + 12.5);
  private static final double MAX_HEIGHT = Units.inchesToMeters(56);
  
  private static final double NINETY_DEG = Math.PI / 2;
  private static final double THETA_MAX = Units.degreesToRadians(42.78);

  private ArmFeedforward armFeedforward;
  private CANSparkMax armMotor;
  private CANSparkMax winchMotor;
  private SparkMaxPIDController armPidController;
  // private SparkMaxPIDController winchPidController;
  private Ultrasonic ultraSonic;
  private AbsoluteEncoder armEncoder;
  private RelativeEncoder winchEncoder;
  private double armFeedforwardValue;
  private double armPosition;
  private double winchPosition;
  private PIDController winchPidController;
  
  public Arm() {
    armFeedforward = new ArmFeedforward(FEED_FORWARD_KS, FEED_FORWARD_KG, FEED_FORWARD_KV, FEED_FORWARD_KA);
    armMotor = ARM_MOTOR.createMotor();
    armPidController = armMotor.getPIDController();
    armPidController.setPositionPIDWrappingEnabled(true);
    armPidController.setPositionPIDWrappingMaxInput(1);
    armPidController.setPositionPIDWrappingMinInput(0);

    armEncoder = armMotor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    armEncoder.setPositionConversionFactor(Units.rotationsToRadians(1));
    
    armPidController.setFeedbackDevice(armEncoder);
    armMotor.getForwardLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen);
    armMotor.getReverseLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyOpen);

    // armPosition = Units.degreesToRadians(90);

    ultraSonic = new Ultrasonic(1,0);
    Ultrasonic.setAutomaticMode(true);
        
    winchMotor = WINCH_MOTOR.createMotor();
    // winchPidController = winchMotor.getPIDController();
    winchEncoder = winchMotor.getEncoder();
    winchPidController = new PIDController(.05, 0, 0);
    // winchPosition = 0;

    // this.setDefaultCommand(this.setPositionCommand(ArmHeight.TRAVELING));
  }

  @Override
  public void periodic() {
    // get the current angle, velocity & acceleration
    var theta = armEncoder.getPosition();
    var velocity = armEncoder.getVelocity();
    var acceleration = 0; // does not seem possible to know

    // calculate the FF
    var armFeedforwardValue = armFeedforward.calculate(Units.rotationsToRadians(theta - .25), velocity);

    // calc max length
    var thetaMod = MathUtil.inputModulus(theta, 0, NINETY_DEG);
    var maxLength = thetaMod < THETA_MAX ? MAX_EXTENSION / Math.cos(thetaMod) : MAX_HEIGHT / Math.sin(thetaMod);

    // winchMotor.setSoftLimit(SoftLimitDirection.kForward, (float) len);
    
    // if (useSmartMotion) {
      // set the reference point
      // 

      // winchMotor.getPIDController().setReference(Math.max(winchPosition, len), ControlType.kSmartMotion);  
    // }
    // else {
    //   armMotor.set(armPosition);
    //   winchMotor.set(winchPosition);
    // }
    
    SmartDashboard.putNumber("Arm position", armEncoder.getPosition());
    SmartDashboard.putNumber("Arm Goal", armPosition);
    SmartDashboard.putNumber("Arm Alt Encoder Velocity", armEncoder.getVelocity());
    SmartDashboard.putNumber("Arm Applied Output", armMotor.getAppliedOutput());
    SmartDashboard.putNumber("Arm calculated FF", armFeedforwardValue);
    SmartDashboard.putNumber("Arm Max Length (m)", maxLength);
    
    SmartDashboard.putNumber("Winch Position (encoder)", winchEncoder.getPosition());
    SmartDashboard.putNumber("Winch Position (ultrasonic)", ultraSonic.getRangeMM());
    SmartDashboard.putNumber("Winch Goal", winchPosition);
    SmartDashboard.putNumber("Winch Velocity", winchEncoder.getVelocity());
    SmartDashboard.putNumber("Winch Applied Output", winchMotor.getAppliedOutput());

    SmartDashboard.putNumber("UltraSonic (in)", getWinchPosition());
    
  }

  private void setArmGoal(double position) {
    armPosition = position;
    armPidController.setReference(position, ControlType.kPosition, 0, armFeedforwardValue, ArbFFUnits.kVoltage);
  }

  private void setWinchGoal(double position) {
    winchPosition = position;
    // TODO: do we need to have a FF?
    // winchPidController.setReference(position, ControlType.kSmartMotion);
    winchMotor.setVoltage(winchPidController.calculate(position, position));
  }
   
  private double getWinchPosition() {
    return ultraSonic.getRangeInches();
  }
   

  public boolean isAtArmGoal() {
    return compareEq(armEncoder.getPosition(), armPosition, 0.05);
  }
  
  public Command setPositionCommand(ArmHeight armHeight) {
    return run(() -> {
      setArmGoal(armHeight.position);
      // set winch goal
    });
  }

  public Command setPositionTestCommand() {
    return run(() -> {
      // Read values from smartdashboard

      // set goals
    });
  }

  public Command runArmTestCommand() {
    return run(() -> {
      armMotor.set(.5d);
      winchMotor.set(.5d);
    });
  }

  public Command manualPositionCommand(DoubleSupplier arm, DoubleSupplier winch) {
    return run(() -> {
      double armVal = MathUtil.applyDeadband(arm.getAsDouble(), 0.09);
      double winchVal = MathUtil.applyDeadband(winch.getAsDouble(), 0.09);

      if (notZero(armVal)) {
        armMotor.set(armVal * .05);
      //   setArmGoal(armEncoder.getPosition() + armVal);
      }
      if (notZero(winchVal)) {
        winchMotor.set(winchVal);
      //   setWinchGoal(winchEncoder.getPosition() + winchVal);
      }
    });
  }

  private boolean notZero(double val) {
    return compareEq(val, 0d, 0.000001d);
  }

  private boolean compareEq(double d1, double d2, double tolerance) {
    return Math.abs(d1 - d2) < tolerance;
  }

  // public void runArm(double value) {
  //   if (Math.abs(value) > .09) {
  //     useSmartMotion = false;
  //     armPosition = value * .5;
  //   //   armMotor.set(armPosition);
  //   }
  //   // else armMotor.set(0);
  //   // // armPosition = 0;
  // }

  // public void runWinch(double value) {
  //   if (Math.abs(value) > .09) {
  //     useSmartMotion = false;
  //     // winchMotor.set(value * .8);
  //     winchPosition = value;
  //   }
  //   // else {
  //   //   winchMotor.set(0);
  //   // }
  // }

  public Command stop() {
    return this.runOnce(() -> {
      winchMotor.stopMotor();
      armMotor.stopMotor();
    });
  }
}
