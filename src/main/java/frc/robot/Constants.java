package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAlternateEncoder.Type;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {    
  
  public static final class DriveConstants {
    public static final int IMU = 1;

    private static final double OPEN_RAMP_RATE = 1.5;
    private static final double WHEEL_DIAMETER_M = Units.inchesToMeters(8);
    private static final double DRIVE_GEAR_RATIO = (70d / 14d) * (66d / 30d);
    private static final double ENCODER_POSITION_CONVERSION =
        Math.PI * WHEEL_DIAMETER_M / DRIVE_GEAR_RATIO;
    // TODO: Find actual values for these!
    private static final double ks = .14802;
    private static final double kv = 2.0852;
    private static final double ka = .45315;
    private static final PIDConfig PID_DEFAULTS = new PIDConfig()
            .kP(0.4) //0.000047148  .21247
            .kI(0)
            .kD(0)
            .kFF(0.000156)
            .maxAcceleration(1500)
            .maxVelocity(5700)
            .outputRangeHigh(1)
            .outputRangeLow(-1)
            .allowedClosedLoopError(1)
            // .minOutputVelocity(minOutputVelocity)
            ;
    public static final MotorConfig FRONT_LEFT = new MotorConfig()
            .canId(10)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .encoderConfig(new EncoderConfig().positionConversionFactor(ENCODER_POSITION_CONVERSION))
            .openLoopRampRate(OPEN_RAMP_RATE)
            ;
    public static final MotorConfig FRONT_RIGHT = new MotorConfig()
            .canId(5)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .encoderConfig(new EncoderConfig().positionConversionFactor(ENCODER_POSITION_CONVERSION))
            .openLoopRampRate(OPEN_RAMP_RATE)
            ;
    public static final MotorConfig REAR_LEFT = new MotorConfig()
            .canId(8)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .encoderConfig(new EncoderConfig().positionConversionFactor(ENCODER_POSITION_CONVERSION))
            .openLoopRampRate(OPEN_RAMP_RATE)
            ;
    public static final MotorConfig REAR_RIGHT = new MotorConfig()
            .canId(9)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .encoderConfig(new EncoderConfig().positionConversionFactor(ENCODER_POSITION_CONVERSION))
            .openLoopRampRate(OPEN_RAMP_RATE)
            
            ;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(21);
    // Distance between centers of front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(14);

    public static final MecanumDriveKinematics DRIVE_KINEMATICS =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    public static final double MAX_SPEED = 5; // m/s
  }

  public static final class ArmConstants {
    // public static final MotorConfig ARM_MOTOR = new MotorConfig()
    //         .canId(12)
    //         .idleMode(IdleMode.kBrake)
    //         .pidConfig(new PIDConfig()
    //           .kP(.88)
    //           .kI(0)
    //           .kD(0)
    //           .kFF(0.0003)
    //           .maxAcceleration(2500)
    //           .maxVelocity(5700)
    //           .outputRangeHigh(1)
    //           .outputRangeLow(-1)
    //           .allowedClosedLoopError(0.02)
    //         )
    //         // .alternateEncoderConfig(new AlternateEncoderConfig())
    //         // .positionConversionFactor(Units.rotationsToRadians(1) / 8192)
    //         // .softLimitForward(new SoftLimit().limit(180)) // TODO: need to find the value to do floor pickup
    //         // .softLimitReverse(new SoftLimit().limit(0))
    //         .openLoopRampRate(1)
    //         .follower(new MotorConfig().canId(13).idleMode(IdleMode.kBrake).inverted(true))
    //         // .startPosition(90)
    //         ;
    public static final double FEED_FORWARD_KS = 0;
    public static final double FEED_FORWARD_KG = 0;
    public static final double FEED_FORWARD_KV = 0;
    public static final double FEED_FORWARD_KA = 0;

    private static final double BASE_LENGTH = Units.inchesToMeters(9);
    private static final double MAX_LENTGH = Units.inchesToMeters(26); 
  //   public static final MotorConfig WINCH_MOTOR = new MotorConfig()
  //     .canId(11)
  //     .idleMode(IdleMode.kBrake)
  //     .pidConfig(new PIDConfig()
  //       .kP(5e-5)
  //       .kI(1e-6)
  //       .kD(0)
  //       .kFF(0.000156)
  //       .maxAcceleration(1500)
  //       .maxVelocity(500)
  //       .outputRangeHigh(1)
  //       .outputRangeLow(-1)
  //       .allowedClosedLoopError(1)
  //     )
  //     // .inverted(true)
  //     // spool diameter * pi * (gear ratio)
  //     // .positionConversionFactor(Units.inchesToMeters(0.787) * Math.PI * (44d /72d))
  //     // .startPosition(BASE_LENGTH)
  //     .softLimitReverse(new SoftLimit().limit((float) BASE_LENGTH))
  //     .softLimitForward(new SoftLimit().limit((float) MAX_LENTGH))
  //     .openLoopRampRate(1);
  }


  public static final class VisionConstants {
    public static final String CAMERA_NAME = "photon";
    public static final Transform3d ROBOT_TO_CAM = new Transform3d(
      new Translation3d(
        0.5, // half meter forward of center
        0.0, // center of robot
        0.5  // half meter up
      ), new Rotation3d(0, 0, 0));
  }

  public static class MotorConfig {
    private int canId;
    private double closedLoopRampRate = 0;
    private double openLoopRampRate = 0;
    private int stallLimit = 60;
    private int freeLimit = 5600;
    // private MotorType type = MotorType.kBrushless;
    private Optional<SoftLimit> softLimitForward = Optional.empty();
    private Optional<SoftLimit> softLimitReverse = Optional.empty();
    private Optional<AlternateEncoderConfig> alternateEncoder = Optional.empty();
    private Optional<EncoderConfig> encoder = Optional.empty();
    private IdleMode idleMode = IdleMode.kBrake;
    private boolean inverted = false;
    
    private List<PIDConfig> pidConfigs = new ArrayList<>();
    private MotorConfig follower;
  
    public CANSparkMax createMotor() {
      return createMotor(true);
    }
  
    private CANSparkMax createMotor(boolean burnFlash) {
      var motor = new CANSparkMax(canId, MotorType.kBrushless);
      motor.restoreFactoryDefaults();
      motor.setClosedLoopRampRate(closedLoopRampRate);
      motor.setIdleMode(idleMode);
      motor.setInverted(inverted);
      // motor.enableVoltageCompensation(nominalVoltage);
      motor.setSmartCurrentLimit(stallLimit, freeLimit);
      motor.setOpenLoopRampRate(openLoopRampRate);
      softLimitForward.ifPresent((softLimit) -> {
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, softLimit.limit);
      });
      softLimitReverse.ifPresent((softLimit) -> {
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kReverse, softLimit.limit);
      });
      // motor.enableVoltageCompensation() ???

      // absoluteEncoder.ifPresent(ae -> {

      // }); 
      // motor.getAbsoluteEncoder(com.revrobotics.SparkMaxAbsoluteEncoder.Type.kDutyCycle).

      alternateEncoder.ifPresent(ae -> {
        RelativeEncoder encoder = motor.getAlternateEncoder(ae.type, ae.countsPerRevolution);
        encoder.setInverted(ae.inverted);
        encoder.setPosition(ae.startPosition);
        // encoder.setPositionConversionFactor(positionConversionFactor);
        // encoder.setVelocityConversionFactor(positionConversionFactor / 60);
        motor.getPIDController().setFeedbackDevice(encoder);
      });
      encoder.ifPresent(e -> {
        motor.getEncoder().setPositionConversionFactor(e.positionConversionFactor);
        motor.getEncoder().setVelocityConversionFactor(e.positionConversionFactor / 60);
        motor.getEncoder().setPosition(e.startPosition);
      });
  
      SparkMaxPIDController pidController = motor.getPIDController();
      for (int i = 0; i < pidConfigs.size(); i++) {
        PIDConfig pidConfig = pidConfigs.get(i);
        pidController.setP(pidConfig.kP, i);
        pidController.setI(pidConfig.kI, i);
        pidController.setD(pidConfig.kD, i);
        pidController.setFF(pidConfig.kFF, i);
        pidController.setOutputRange(
            pidConfig.outputRangeLow, pidConfig.outputRangeHigh, i);
        pidController.setSmartMotionAllowedClosedLoopError(pidConfig.allowedClosedLoopError, i);
        pidController.setSmartMotionMaxAccel(pidConfig.maxAcceleration, i);
        pidController.setSmartMotionMaxVelocity(pidConfig.maxVelocity, i);
        pidController.setSmartMotionMinOutputVelocity(pidConfig.minOutputVelocity, i);
      }
  
      if (follower != null) {
        boolean followerInverted = follower.inverted;
        follower.inverted = this.inverted;
  
        var followerMotor = follower.createMotor(false);
        followerMotor.follow(motor, followerInverted);
        if (burnFlash) {
          followerMotor.burnFlash();
        }
      }
  
      if (burnFlash) {
        motor.burnFlash();
      }
      return motor;
    }

    public MotorConfig canId(int canId) {
      this.canId = canId;
      return this;
    }

    public MotorConfig closedLoopRampRate(double closedLoopRampRate) {
      this.closedLoopRampRate = closedLoopRampRate;
      return this;
    }

    public MotorConfig openLoopRampRate(double openLoopRampRate) {
      this.openLoopRampRate = openLoopRampRate;
      return this;
    }

    public MotorConfig softLimitForward(SoftLimit softLimitForward) {
      this.softLimitForward = Optional.of(softLimitForward);
      return this;
    }

    public MotorConfig softLimitReverse(SoftLimit softLimitReverse) {
      this.softLimitReverse = Optional.of(softLimitReverse);
      return this;
    }

    public MotorConfig idleMode(IdleMode idleMode) {
      this.idleMode = idleMode;
      return this;
    }

    public MotorConfig inverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }

    public MotorConfig pidConfigs(List<PIDConfig> pidConfigs) {
      this.pidConfigs = pidConfigs;
      return this;
    }

    public MotorConfig follower(MotorConfig follower) {
      this.follower = follower;
      return this;
    }

    public MotorConfig pidConfig(PIDConfig pidConfig) {
      pidConfigs.add(pidConfig);
      return this;
    }

    public MotorConfig alternateEncoderConfig(AlternateEncoderConfig alternateEncoderConfig) {
      this.alternateEncoder = Optional.ofNullable(alternateEncoderConfig);
      return this;
    }

    public MotorConfig encoderConfig(EncoderConfig encoderConfig) {
      this.encoder = Optional.ofNullable(encoderConfig);
      return this;
    }
  }
  
  static final class PIDConfig {
    private double kP;
    private double kI;
    private double kD;
    private double kFF;
    private double outputRangeLow = -1d;
    private double outputRangeHigh = 1d;
    private double allowedClosedLoopError = 0;
    private double maxAcceleration;
    private double maxVelocity;
    private double minOutputVelocity;

    public PIDConfig kP(double kP) {
      this.kP = kP;
      return this;
    }
    public PIDConfig kI(double kI) {
      this.kI = kI;
      return this;
    }
    public PIDConfig kD(double kD) {
      this.kD = kD;
      return this;
    }
    public PIDConfig kFF(double kFF) {
      this.kFF = kFF;
      return this;
    }
    public PIDConfig outputRangeLow(double outputRangeLow) {
      this.outputRangeLow = outputRangeLow;
      return this;
    }
    public PIDConfig outputRangeHigh(double outputRangeHigh) {
      this.outputRangeHigh = outputRangeHigh;
      return this;
    }
    public PIDConfig allowedClosedLoopError(double allowedClosedLoopError) {
      this.allowedClosedLoopError = allowedClosedLoopError;
      return this;
    }
    public PIDConfig maxAcceleration(double maxAcceleration) {
      this.maxAcceleration = maxAcceleration;
      return this;
    }
    public PIDConfig maxVelocity(double maxVelocity) {
      this.maxVelocity = maxVelocity;
      return this;
    }
    public PIDConfig minOutputVelocity(double minOutputVelocity) {
      this.minOutputVelocity = minOutputVelocity;
      return this;
    }
  }
  
  static final class SoftLimit {
    private float limit;

    public SoftLimit limit(float limit) {
      this.limit = limit;
      return this;
    }
  }

  static final class EncoderConfig {
    private double positionConversionFactor = 1;
    private double startPosition = 0;

    public EncoderConfig startPosition(double startPosition) {
      this.startPosition = startPosition;
      return this;
    }

    public EncoderConfig positionConversionFactor(double positionConversionFactor) {
      this.positionConversionFactor = positionConversionFactor;
      return this;
    }
  }

  static final class AlternateEncoderConfig {
    private int countsPerRevolution = 8192;
    private Type type = Type.kQuadrature;
    private boolean inverted = false;
    private double positionConversionFactor = 1;
    private double startPosition = 0;

    public AlternateEncoderConfig startPosition(double startPosition) {
      this.startPosition = startPosition;
      return this;
    }

    public AlternateEncoderConfig positionConversionFactor(double positionConversionFactor) {
      this.positionConversionFactor = positionConversionFactor;
      return this;
    }

    public AlternateEncoderConfig type(Type type) {
      this.type = type;
      return this;
    }

    public AlternateEncoderConfig countsPerRevolution(int countsPerRevolution) {
      this.countsPerRevolution = countsPerRevolution;
      return this;
    }

    public AlternateEncoderConfig inverted(boolean inverted) {
      this.inverted = inverted;
      return this;
    }
  }
}
