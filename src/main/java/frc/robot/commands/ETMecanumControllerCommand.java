package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Edwardsville Technologies version of the <code>MecanumControllerCommand</code>. This version is
 * built to use the holonomic trajectory information available from the PathPlannerTrajectory
 * library. It also assumes that smart motor controllers are being used for the drivetrain to handle
 * PID controls.
 */
public class ETMecanumControllerCommand extends PPMecanumControllerCommand {
  public ETMecanumControllerCommand(
        PathPlannerTrajectory trajectory,
        Supplier<Pose2d> poseSupplier,
        MecanumDriveKinematics kinematics,
        double maxWheelVelocityMetersPerSecond,
        Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds,
        boolean useAllianceColor,
        Subsystem... requirements) {

    super(trajectory, poseSupplier, kinematics, null, null, null,
      maxWheelVelocityMetersPerSecond, outputWheelSpeeds, useAllianceColor, requirements);
  }
}
