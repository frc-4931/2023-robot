package frc.robot.subsystems;

import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KA;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KG;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KS;
import static frc.robot.Constants.ArmConstants.FEED_FORWARD_KV;
import static frc.robot.Constants.ArmConstants.ARM_MOTOR;
import static frc.robot.Constants.ArmConstants.WINCH_MOTOR;

import java.util.function.DoubleSupplier;

import javax.security.auth.PrivateCredentialPermission;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.SparkMaxAlternateEncoder.Type;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
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
  private SparkMaxPIDController winchPidController;
  private Ultrasonic trueUltraSonic;
  private AnalogInput ultraSonic;
  // private RelativeEncoder armEncoder;
  private AbsoluteEncoder armEncoder;
  private RelativeEncoder winchEncoder;
  private double armFeedforwardValue;
  private double armPosition;
  private double winchPosition;
  private DigitalInput[] throughBeams;
  
  public Arm() {
    armFeedforward = new ArmFeedforward(FEED_FORWARD_KS, FEED_FORWARD_KG, FEED_FORWARD_KV, FEED_FORWARD_KA);
    armMotor = ARM_MOTOR.createMotor();
    armPidController = armMotor.getPIDController();
    ultraSonic = new AnalogInput(2);
    trueUltraSonic = new Ultrasonic(1,0);
    Ultrasonic.setAutomaticMode(true);
    armEncoder = armMotor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle);
    armEncoder.setPositionConversionFactor(Units.rotationsToRadians(1));
    armPidController.setFeedbackDevice(armEncoder);
    throughBeams = new DigitalInput[] {new DigitalInput(9), new DigitalInput(4)};

    // armPosition = Units.degreesToRadians(90);
        
    winchMotor = WINCH_MOTOR.createMotor();
    winchPidController = winchMotor.getPIDController();
    winchEncoder = winchMotor.getEncoder();
    // winchPosition = 0;

    this.setDefaultCommand(this.setPositionCommand(ArmHeight.TRAVELING));
  }

  @Override
  public void periodic() {
    // get the current angle, velocity & acceleration
    var theta = armEncoder.getPosition();
    var velocity = armEncoder.getVelocity();
    var acceleration = 0; // does not seem possible to know

    // calculate the FF
    var armFeedforwardValue = armFeedforward.calculate(Units.rotationsToRadians(theta), velocity);

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
    
    SmartDashboard.putNumber("Winch Position", winchEncoder.getPosition());
    SmartDashboard.putNumber("Winch Goal", winchPosition);
    SmartDashboard.putNumber("Winch Velocity", winchEncoder.getVelocity());
    SmartDashboard.putNumber("Winch Applied Output", winchMotor.getAppliedOutput());

    SmartDashboard.putNumber("Ultrasonic Voltage", ultraSonic.getVoltage());
    SmartDashboard.putNumber("UltraSonic Value", ultraSonic.getValue());
    SmartDashboard.putNumber("UltraSonicCalculated", getWinchPosition());
    SmartDashboard.putNumber("TrueUltraSonicCalculated", getWinchPosition2());
    SmartDashboard.putNumber("TrueUltraSonicMillimeters", trueUltraSonic.getRangeMM());

    SmartDashboard.putBoolean("through sensor channel9", throughBeams[0].get());
    SmartDashboard.putBoolean("through sensor channel4", throughBeams[1].get());
  }

  private void setArmGoal(double position) {
    armPosition = position;
    armPidController.setReference(position, ControlType.kSmartMotion, 0, armFeedforwardValue, ArbFFUnits.kVoltage);
  }

  private void setWinchGoal(double position) {
    winchPosition = position;
    // TODO: do we need to have a FF?
    winchPidController.setReference(position, ControlType.kSmartMotion);
  }
   private double getWinchPosition() {
    double ultraSonicValue = ultraSonic.getValue() * 4.88 / 5;
    return ultraSonicValue;
  }
  private double getWinchPosition2() {
    double trueUltraSonicValue = trueUltraSonic.getRangeInches();
    return trueUltraSonicValue;
  }
   

  public boolean isAtArmGoal() {
    return compare(armEncoder.getPosition(), armPosition, 0.05);
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
        // System.out.printf("armVal %f%n", armVal);
        armMotor.set(armVal);
      //   setArmGoal(armEncoder.getPosition() + armVal);
      }
      if (notZero(winchVal)) {
        winchMotor.set(winchVal);
      //   setWinchGoal(winchEncoder.getPosition() + winchVal);
      }
    });
  }

  private boolean notZero(double val) {
    return compare(val, 0d, 0.000001d);
  }

  private boolean compare(double d1, double d2, double tolerance) {
    return Math.abs(d1 - d2) > tolerance;
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
