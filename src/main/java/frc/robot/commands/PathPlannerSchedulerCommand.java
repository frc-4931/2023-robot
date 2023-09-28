package frc.robot.commands;

import java.util.function.Consumer;
import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPMecanumControllerCommand;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class PathPlannerSchedulerCommand extends CommandBase {
  // private final PathConstraints pathConstraints;
  // private final Supplier<Pose2d> poseSupplier;
  // private final MecanumDriveKinematics kinematics;
  // private final Consumer<MecanumDriveWheelSpeeds> outputWheelSpeeds;
  // private final Subsystem[] requirements;

  @Override
  public void initialize() {
    // read values

    // PathPlannerTrajectory trajectory = PathPlanner.generatePath(pathConstraints, null);;

    // new PPMecanumControllerCommand(trajectory, poseSupplier, kinematics,
    //   null, null, null, 
    //   0, outputWheelSpeeds, requirements).schedule();
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
