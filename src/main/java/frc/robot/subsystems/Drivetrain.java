package frc.robot.subsystems;

import static frc.robot.Constants.DriveConstants.DRIVE_KINEMATICS;
import static frc.robot.Constants.DriveConstants.FRONT_LEFT;
import static frc.robot.Constants.DriveConstants.FRONT_RIGHT;
import static frc.robot.Constants.DriveConstants.IMU;
import static frc.robot.Constants.DriveConstants.MAX_SPEED;
import static frc.robot.Constants.DriveConstants.REAR_LEFT;
import static frc.robot.Constants.DriveConstants.REAR_RIGHT;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax rearLeftMotor;
  private CANSparkMax rearRightMotor;
  private MecanumDrive drive;
  private Field2d field2d;
  private MecanumDrivePoseEstimator poseEstimator;
  private WPI_PigeonIMU imu;
  // private AHRS ahrs;
  private boolean fieldOriented = false;
  private static final Rotation2d ZERO = new Rotation2d(0);
  private double multiplier = 1.0;

  public Drivetrain() {
    frontLeftMotor = FRONT_LEFT.createMotor();
    frontRightMotor = FRONT_RIGHT.createMotor();
    rearLeftMotor = REAR_LEFT.createMotor();
    rearRightMotor = REAR_RIGHT.createMotor();

    // initialize the gyro
    imu = new WPI_PigeonIMU(IMU);
    imu.configFactoryDefault();
    imu.setYaw(0);
    imu.setAccumZAngle(0);
    imu.reset();

    // ahrs = new AHRS(Port.kMXP);

    drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    poseEstimator = new MecanumDrivePoseEstimator(DRIVE_KINEMATICS, getAngle(), getWheelPositions(), new Pose2d());
    field2d = new Field2d();
  }

  public void drive(double xSpeed, double ySpeed, double zSpeed) {
    var z = fieldOriented ? getAngle() : ZERO;
    drive.driveCartesian(xSpeed * multiplier, ySpeed * multiplier, zSpeed * multiplier, z);
  }

  public void outputWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
    wheelSpeeds.desaturate(MAX_SPEED);
    // TODO: need arbitrary FF?
    frontLeftMotor.getPIDController().setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kVelocity);
    frontRightMotor.getPIDController().setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kVelocity);
    rearLeftMotor.getPIDController().setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kVelocity);
    rearRightMotor.getPIDController().setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kVelocity);
  }

  public void outputChasisSpeeds(ChassisSpeeds chassisSpeeds) {
    outputWheelSpeeds(DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds));
  }

  public void resetPose(Pose2d pose) {
    System.out.printf("reset the pose angle: %f; pose: %f %f%n", getAngle().getDegrees(), pose.getX(), pose.getY());
    poseEstimator.resetPosition(getAngle(), getWheelPositions(), pose);
  }

  public void updatePose(Pose2d calcPose2d, double timestamp) {
    poseEstimator.addVisionMeasurement(calcPose2d, timestamp);
  }

  private void updateOdometry() {
    poseEstimator.update(getAngle(), getWheelPositions());
  }

  public Rotation2d getAngle() {
    return imu.getRotation2d();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(frontLeftMotor.getEncoder().getPosition(),
        frontRightMotor.getEncoder().getPosition(), rearLeftMotor.getEncoder().getPosition(),
        rearRightMotor.getEncoder().getPosition());
  }

  public MecanumDriveWheelSpeeds getWheelSpeeds() {
    return new MecanumDriveWheelSpeeds(frontLeftMotor.getEncoder().getVelocity(),
        frontRightMotor.getEncoder().getVelocity(), rearLeftMotor.getEncoder().getVelocity(),
        rearRightMotor.getEncoder().getVelocity());
  }

  public ChassisSpeeds getChassisSpeeds() {
    return DRIVE_KINEMATICS.toChassisSpeeds(getWheelSpeeds());
  }

  public boolean isFieldOriented() {
    return fieldOriented;
  }

  @Override
  public void periodic() {
    updateOdometry();
    field2d.setRobotPose(getPose());
    SmartDashboard.putData(drive);
    SmartDashboard.putData(imu);
    // SmartDashboard.putData(field2d);
    SmartDashboard.putBoolean("FieldOriented", fieldOriented);
    SmartDashboard.putNumber("DriveMultiplier", multiplier);
    
    // SmartDashboard.putData(ahrs);
  }

  public Command defaultDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier zSupplier) {
    return this.run(() -> drive(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
        zSupplier.getAsDouble()));
  }

  public Command wheelSpeedDriveCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier,
      DoubleSupplier zSupplier) {
    return this.run(() -> {
      var wheelSpeeds = DRIVE_KINEMATICS.toWheelSpeeds(
          fieldOriented
              ? ChassisSpeeds.fromFieldRelativeSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(),
                  zSupplier.getAsDouble(), getAngle())
              : new ChassisSpeeds(xSupplier.getAsDouble(), ySupplier.getAsDouble(), zSupplier.getAsDouble()));
      outputWheelSpeeds(wheelSpeeds);
    });
  }

  public Command toggleFieldOriented() {
    return this.runOnce(() -> fieldOriented = !fieldOriented);
  }

  public Command lowerMultiplier() {
    return this.runOnce(() -> multiplier = Math.max(.25, multiplier - .25));
  }

  public Command raiseMultiplier() {
    return this.runOnce(() -> multiplier = Math.min(1, multiplier + .25));
  }

  public Command resetGyro() {
    return this.runOnce(() -> this.imu.reset());
  }

  // public Command followTrajectoryCommand(PathPlannerTrajectory traj, boolean
  // isFirstPath) {
  // return new SequentialCommandGroup(
  // new InstantCommand(() -> {
  // // Reset odometry for the first path you run during auto
  // if(isFirstPath){
  // this.resetPose(traj.getInitialHolonomicPose());
  // }
  // }),
  // new PPMecanumControllerCommand(
  // traj,
  // this::getPose, // Pose supplier
  // DRIVE_KINEMATICS, // MecanumDriveKinematics
  // new PIDController(0, 0, 0), // X controller. Tune these values for your
  // robot. Leaving them 0 will only use feedforwards.
  // new PIDController(0, 0, 0), // Y controller (usually the same values as X
  // controller)
  // new PIDController(0, 0, 0), // Rotation controller. Tune these values for
  // your robot. Leaving them 0 will only use feedforwards.
  // 3.0, // Max wheel velocity meters per second
  // this::outputWheelSpeeds, // MecanumDriveWheelSpeeds consumer
  // true, // Should the path be automatically mirrored depending on alliance
  // color. Optional, defaults to true
  // this // Requires this drive subsystem
  // )
  // );
  // }

  // private void driveTo(Pose2d pose) {
  // pose.interpolate(pose, kTrackWidth)
  // }

  // private void stuff() {
  // PIDController pidController;
  // pidController.calculate(kTrackWidth, IMU)
  // }
}
