package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.MecanumDriveOdometry;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.DriveConstants.*;

public class Drivetrain extends SubsystemBase {
  private CANSparkMax frontLeftMotor;
  private CANSparkMax frontRightMotor;
  private CANSparkMax rearLeftMotor;
  private CANSparkMax rearRightMotor;
  private MecanumDrive drive;
  private MecanumDriveOdometry odometry;
  private static final Rotation2d ZERO = new Rotation2d(0);

  public Drivetrain() {
    frontLeftMotor = FRONT_LEFT.createMotor();
    frontRightMotor = FRONT_RIGHT.createMotor();
    rearLeftMotor = REAR_LEFT.createMotor();
    rearRightMotor = REAR_RIGHT.createMotor();

    // initialize the gyro

    drive = new MecanumDrive(frontLeftMotor, rearLeftMotor, frontRightMotor, rearRightMotor);
    odometry = new MecanumDriveOdometry(DRIVE_KINEMATICS, getAngle(), getWheelPositions());

  }

  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOriented) {
    var z = fieldOriented ? getAngle() : ZERO;
    drive.driveCartesian(xSpeed, ySpeed, zSpeed, z);
  }

  public void resetOdemetry(Pose2d pose) {
    odometry.resetPosition(getAngle(), null, pose);
  }

  private Rotation2d getAngle() {
    // FIXME:
    return ZERO;
  }

  public MecanumDriveWheelPositions getWheelPositions() {
    return new MecanumDriveWheelPositions(frontLeftMotor.getEncoder().getPosition(),
      frontRightMotor.getEncoder().getPosition(), rearLeftMotor.getEncoder().getPosition(), 
      rearRightMotor.getEncoder().getPosition());
  }
}
