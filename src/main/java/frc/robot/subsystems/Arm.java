package frc.robot.subsystems;

// import static frc.robot.Constants.ArmConstants.ARM_MOTOR;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KA;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KG;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KS;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KV;
// import static frc.robot.Constants.ArmConstants.WINCH_MOTOR;

import java.util.function.DoubleSupplier;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAnalogSensor.Mode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.SparkMotorConfig;

public class Arm extends SubsystemBase {
  public enum ArmPosition {
    TIER_1(0, MIN_LEN), CONE_2(.66393, 490), 
    CUBE_1(0, MIN_LEN), CONE_3(0, MIN_LEN), 
    CUBE_2(0, MIN_LEN), LOADING(0.4, MIN_LEN), 
    UP(0.5, MIN_LEN);

    double angle;
    double armLength;
    ArmPosition(double angle, double armLength) {
      this.angle = angle;
      this.armLength = armLength;
    }
  }
  private static final double MIN_LEN = 182.2;
  private static final double MAX_LEN = 662.0;
  private static final double MAX_EXTENSION = Units.inchesToMeters(48 + 12.5);
  // private static final double MAX_HEIGHT = Units.inchesToMeters(56);
  
  // private static final double NINETY_DEG = Math.PI / 2;
  // private static final double THETA_MAX = Units.degreesToRadians(42.78);

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
  // private DigitalInput digitalInput = new DigitalInput(9);
  private static double WINCH_CONVERSION = Units.inchesToMeters(0.787) * Math.PI * (44d /72d); 
  
  public Arm() {
    armFeedforward = new ArmFeedforward(FEED_FORWARD_KS, FEED_FORWARD_KG, FEED_FORWARD_KV, FEED_FORWARD_KA);
    armMotor = SparkMotorConfig.motor(12, IdleMode.kBrake, false);
    // CANSparkMax extra = 
    SparkMotorConfig.motor(13, IdleMode.kBrake, true).follow(armMotor, true);

    // ARM_MOTOR.createMotor();
    armPidController = SparkMotorConfig.pidController(armMotor, .88d, 0d, 0d, 0.0003, 5700d, 2500d, -1d, 1d, .02d);
    
    // armMotor.getPIDController();
    armPidController.setPositionPIDWrappingEnabled(true);
    armPidController.setPositionPIDWrappingMaxInput(1);
    armPidController.setPositionPIDWrappingMinInput(0);
    
    armEncoder = armMotor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    // armEncoder.setPositionConversionFactor(Units.rotationsToRadians(1));
    resetArmEncoder();

    armPidController.setFeedbackDevice(armEncoder);
    armMotor.getForwardLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyClosed);
    armMotor.getReverseLimitSwitch(com.revrobotics.SparkMaxLimitSwitch.Type.kNormallyClosed);
    armMotor.setSmartCurrentLimit(10);
    
    
    // armPosition = Units.degreesToRadians(90);

    ultraSonic = new Ultrasonic(1,0);
    Ultrasonic.setAutomaticMode(true);
        
    winchMotor = SparkMotorConfig.motor(11, IdleMode.kBrake, false);
    winchEncoder = winchMotor.getEncoder();
    // winchEncoder.setInverted(true);
    winchEncoder.setPosition(0);
    winchMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    winchMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // (float) (Units.inchesToMeters(-2.31) * WINCH_CONVERSION)
    winchMotor.setSoftLimit(SoftLimitDirection.kReverse, -8.0f);
    winchMotor.setSoftLimit(SoftLimitDirection.kForward, 0);
    // winchMotor.setSoftLimit(SoftLimitDirection.kForward, (float) ((MAX_LEN - MIN_LEN) * 1000 * WINCH_CONVERSION));
    
    // WINCH_MOTOR.createMotor();
    // winchPidController = winchMotor.getPIDController();
    // winchEncoder = winchMotor.getEncoder();

    winchPidController = new PIDController(.4, 0, 0); //.7
    // winchPosition = 0;

    // this.setDefaultCommand(this.setPositionCommand(ArmPosition.UP));
    SmartDashboard.putData("Arm Up", setPositionCommand(ArmPosition.UP));
    SmartDashboard.putData("Reset Arm Encoder", resetArmEncoderCommand());
  }

  @Override
  public void periodic() {
    // SmartDashboard.putBoolean("dio", digitalInput.get());

    // get the current angle, velocity & acceleration
    var theta = armEncoder.getPosition();
    var velocity = armEncoder.getVelocity();
    var acceleration = 0; // does not seem possible to know

    // calculate the FF
    var armFeedforwardValue = armFeedforward.calculate(Units.rotationsToRadians(theta - .25), velocity);

    // calc max length
    // var thetaMod = MathUtil.inputModulus(theta, 0, NINETY_DEG);
    // var maxLength = thetaMod < THETA_MAX ? MAX_EXTENSION / Math.cos(thetaMod) : MAX_HEIGHT / Math.sin(thetaMod);

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
    // SmartDashboard.putNumber("Arm Max Length (m)", maxLength);
    SmartDashboard.putBoolean("Forward limit", armMotor.getFault(FaultID.kHardLimitFwd));
    SmartDashboard.putBoolean("Reverse limit", armMotor.getFault(FaultID.kHardLimitRev));

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
    // winchMotor.setVoltage(winchPidController.calculate(position, position));
    winchMotor.set(winchPidController.calculate(position));
  }
   
  private double getWinchPosition() {
    return ultraSonic.getRangeInches();
  }
   

  public boolean isAtArmGoal() {
    return compareEq(armEncoder.getPosition(), armPosition, 0.05);
  }
  
  public CommandBase setPositionCommand(ArmPosition armGoal) {  
    return 
    run(() -> {
      setArmGoal(armGoal.angle);
      setWinchGoal(ultraSonic.getRangeMM());
    }).beforeStarting(() -> {
      winchPidController.reset();
      winchPidController.setSetpoint(armGoal.armLength);
    }, this);
  }

  private void resetArmEncoder() {
    System.out.printf("Enc start pos %f%n", armEncoder.getPosition());
    armEncoder.setZeroOffset(MathUtil.clamp(armEncoder.getPosition() + .5, 0, 1));
  }

  public CommandBase resetArmEncoderCommand() {
    return runOnce(this::resetArmEncoder).ignoringDisable(true);
  }

  public CommandBase setPositionTestCommand() {
    return run(() -> {
      // Read values from smartdashboard

      // set goals
    });
  }

  public CommandBase runArmTestCommand() {
    return run(() -> {
      armMotor.set(.5d);
      winchMotor.set(.5d);
    });
  }

  public CommandBase manualPositionCommand(DoubleSupplier arm, DoubleSupplier winch, 
  boolean useArm, DoubleSupplier throttle) {
    return run(() -> {
      double armVal = (useArm || throttle.getAsDouble() > .7) ? MathUtil.applyDeadband(arm.getAsDouble(), 0.09) : 0;
      double winchVal = (!useArm || throttle.getAsDouble() > .7) ? MathUtil.applyDeadband(winch.getAsDouble(), 0.09) : 0;

      armMotor.set(armVal * .35);
      var max = winchVal > 0 ? 0.18 : 0.10; 
      winchMotor.set(winchVal * max);
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
