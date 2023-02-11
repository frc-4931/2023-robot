package frc.robot.util;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class PathPlannerUtil {

  public enum Spot {
    SPOT_1, SPOT_2, SPOT_3, SPOT_4, SPOT_5, SPOT_6, SPOT_7, SPOT_8, SPOT_9
  }
  public enum Row {
    BOTTOM, MIDDLE
  }
  public enum Side {
    LEFT, RIGHT
  }

  private Spot spot;
  private Row row;
  private Side side;
  private boolean useIntermediate;

  public PathPlannerUtil() {
    ShuffleboardTab tab = Shuffleboard.getTab("PathPlanner");
    tab.addString("Spot", spot::name);
    tab.addString("Row", row::name);
    tab.addString("Entry Side", side::name);
    tab.addBoolean("Use Extra Step", () -> useIntermediate);
    tab.addBoolean("Ready?", this::isReady);
  }

  public Optional<PathPlannerTrajectory> generateTrajectory(PathConstraints pathConstraints, Pose2d currentPose, ChassisSpeeds chassisSpeeds) {
    if (isReady()) {
      List<PathPoint> pathPoints = new ArrayList<>();
      pathPoints.add(PathPoint.fromCurrentHolonomicState(currentPose, chassisSpeeds));
      if (useIntermediate) {
        // TODO: add a point close to the 
      }
      pathPoints.add(new PathPoint(null, null, null));
      PathPlannerTrajectory trajectory = PathPlanner.generatePath(pathConstraints, pathPoints);
      return Optional.of(trajectory);
    }
    return Optional.empty();
  }
  // @Override
  // public void initSendable(SendableBuilder builder) {
  //   super.initSendable(builder);
  //   builder.addStringProperty("Spot", () -> spot.name(), (s) -> setSpot(Spot.valueOf(s)));
  //   builder.addStringProperty("Row", () -> row.name(), (r) -> setRow(Row.valueOf(r)));
  //   builder.addStringProperty("Side", () -> side.name(), (s) -> setSide(Side.valueOf(s)));
  //   builder.addBooleanProperty("UseIntermediate", () -> useIntermediate, this::setUseIntermediate);
  //   builder.addBooleanProperty("Ready", this::isReady, null);
  // }

  public void setSpot(Spot spot) {
    this.spot = spot;
  }

  public void setRow(Row row) {
    this.row = row;
  }

  public void setSide(Side side) {
    this.side = side;
  }

  public void setUseIntermediate(boolean useIntermediate) {
    this.useIntermediate = useIntermediate;
  }

  public boolean isReady() {
    // TODO: we can validate that the spot matches the captured game element?
    return spot != null && row != null && side != null;
  }

  // public Command driveCommand() {
  //   

  //   new PPMecanumControllerCommand(trajectory, poseSupplier, kinematics,
  //     null, null, null, 
  //     0, outputWheelSpeeds, requirements);
  // }

}
