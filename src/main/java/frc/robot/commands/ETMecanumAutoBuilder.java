package frc.robot.commands;

import java.util.Map;
import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.BaseAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class ETMecanumAutoBuilder extends BaseAutoBuilder {
  private MecanumDriveKinematics kinematics;
  private double maxWheelVelocityMetersPerSecond;
  private Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds;
  private Subsystem[] driveRequirements;

  public ETMecanumAutoBuilder(Supplier<Pose2d> poseSupplier,
  Consumer<Pose2d> resetPose,
  MecanumDriveKinematics kinematics,
  double maxWheelVelocityMetersPerSecond,
  Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
  Map<String, Command> eventMap,
  boolean useAllianceColor,
  Subsystem... requirements) {

    super(poseSupplier, resetPose, eventMap, DrivetrainType.HOLONOMIC, useAllianceColor);
    this.kinematics = kinematics;
    this.maxWheelVelocityMetersPerSecond = maxWheelVelocityMetersPerSecond;
    this.outputWheelSpeeds = outputWheelSpeeds;
    this.driveRequirements = requirements;
  }

  @Override
  public CommandBase followPath(PathPlannerTrajectory trajectory) {
    return new ETMecanumControllerCommand(trajectory, poseSupplier, kinematics, 
    maxWheelVelocityMetersPerSecond, outputWheelSpeeds, useAllianceColor, driveRequirements);
  }
  
}
