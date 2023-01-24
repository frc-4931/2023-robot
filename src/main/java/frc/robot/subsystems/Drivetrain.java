package frc.robot.subsystems;

import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private static final Rotation2d ZERO = new Rotation2d(0);

  // 4 motors
  // imu
  private boolean fieldOriented = false;
  private MecanumDrive mecanumDrive;
  private Field2d field2d;
  private MecanumDrivePoseEstimator poseEstimator;

  public Drivetrain() {
    // mecanumDrive = new MecanumDrive(null, null, null, null)

    poseEstimator = new MecanumDrivePoseEstimator(null, ZERO, null, null);
  }

  public Rotation2d getAngle() {
    return null;
  }

  public void drive(double xSpeed, double ySpeed, double zSpeed) {
    Rotation2d angle = fieldOriented ? getAngle() : ZERO;
    mecanumDrive.driveCartesian(xSpeed, ySpeed, zSpeed, angle);
  }

  public void setFieldOriented(boolean fieldOriented) {
    this.fieldOriented = fieldOriented;
  }

  public void toggleFieldOriented() {
    fieldOriented = !fieldOriented;
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(getAngle(), getWheelPositions(), pose);
  }

  public void updatePose(Pose2d calcPose2d, long timestamp) {
    poseEstimator.addVisionMeasurement(calcPose2d, timestamp);
  }

  private void updatePose() {
    poseEstimator.update(getAngle(), getWheelPositions());
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    //FIXME:
    return null;
  }

  // public void outputWheelSpeeds(MecanumDriveWheelSpeeds wheelSpeeds) {
  //   wheelSpeeds.desaturate(MAX_SPEED);
  //   // TODO: need arbitrary FF?
  //   frontLeftMotor.getPIDController().setReference(wheelSpeeds.frontLeftMetersPerSecond, ControlType.kSmartVelocity);
  //   frontRightMotor.getPIDController().setReference(wheelSpeeds.frontRightMetersPerSecond, ControlType.kSmartVelocity);
  //   rearLeftMotor.getPIDController().setReference(wheelSpeeds.rearLeftMetersPerSecond, ControlType.kSmartVelocity);
  //   rearRightMotor.getPIDController().setReference(wheelSpeeds.rearRightMetersPerSecond, ControlType.kSmartVelocity);
  // }

  // public void outputChasisSpeeds(ChassisSpeeds chassisSpeeds) {
  //   outputWheelSpeeds(DRIVE_KINEMATICS.toWheelSpeeds(chassisSpeeds));    
  // }
}
