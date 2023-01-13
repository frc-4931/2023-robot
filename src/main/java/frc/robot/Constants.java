package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.util.Units;

public final class Constants {    
  
  public static final class DriveConstants {
    private static final double OPEN_RAMP_RATE = 0.75;
    private static final double WHEEL_DIAMETER_M = Units.inchesToMeters(8);
    private static final double DRIVE_GEAR_RATIO = (70d / 14d) * (66d / 30d);
    private static final double ENCODER_POSITION_CONVERSION =
        Math.PI * WHEEL_DIAMETER_M / DRIVE_GEAR_RATIO;
    // TODO: Find actual values for these!
    private static final PIDConfig PID_DEFAULTS = new PIDConfig()
            .kP(5e-5)
            .kI(1e-6)
            .kD(0)
            .kFF(0.000156)
            .maxAcceleration(1500)
            .maxVelocity(2000)
            .outputRangeHigh(1)
            .outputRangeLow(-1)
            .allowedClosedLoopError(1)
            // .minOutputVelocity(minOutputVelocity)
            ;
    public static final MotorConfig FRONT_LEFT = new MotorConfig()
            .canId(10)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            ;
    public static final MotorConfig FRONT_RIGHT = new MotorConfig()
            .canId(5)
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            ;
    public static final MotorConfig REAR_LEFT = new MotorConfig()
            .canId(8)
            .idleMode(IdleMode.kBrake)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            ;
    public static final MotorConfig REAR_RIGHT = new MotorConfig()
            .canId(9)
            .idleMode(IdleMode.kBrake)
            .inverted(true)
            .pidConfig(PID_DEFAULTS)
            .positionConversionFactor(ENCODER_POSITION_CONVERSION)
            .openLoopRampRate(OPEN_RAMP_RATE)
            ;

    // Distance between centers of right and left wheels on robot
    public static final double kTrackWidth = Units.inchesToMeters(22.09);
    // Distance between centers of front and back wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(14);

    public static final MecanumDriveKinematics DRIVE_KINEMATICS =
        new MecanumDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));
  }


  public static class MotorConfig {
    private int canId;
    private double closedLoopRampRate = 0;
    private double openLoopRampRate = 0;
    // private MotorType type = MotorType.kBrushless;
    private Optional<SoftLimit> softLimitForward = Optional.empty();
    private Optional<SoftLimit> softLimitReverse = Optional.empty();
    private IdleMode idleMode = IdleMode.kBrake;
    private boolean inverted = false;
    private double positionConversionFactor = 1;
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
      // motor.setSmartCurrentLimit(stallLimit, freeLimit)
      motor.setOpenLoopRampRate(openLoopRampRate);
      softLimitForward.ifPresent((softLimit) -> {
        motor.enableSoftLimit(SoftLimitDirection.kForward, true);
        motor.setSoftLimit(SoftLimitDirection.kForward, softLimit.limit);
      });
      softLimitReverse.ifPresent((softLimit) -> {
        motor.enableSoftLimit(SoftLimitDirection.kReverse, true);
        motor.setSoftLimit(SoftLimitDirection.kReverse, softLimit.limit);
      });

      motor.getEncoder().setPositionConversionFactor(positionConversionFactor);
      motor.getEncoder().setVelocityConversionFactor(positionConversionFactor / 60);
      motor.getEncoder().setPosition(0);
  
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
        followerMotor.burnFlash();
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

    public MotorConfig positionConversionFactor(double positionConversionFactor) {
      this.positionConversionFactor = positionConversionFactor;
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

}
