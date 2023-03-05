// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.MecanumAutoBuilder;
import com.pathplanner.lib.auto.PIDConstants;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.util.NWUXboxController;
import frc.robot.util.PathPlannerUtil;

public class RobotContainer {
  private CommandXboxController xboxController;
  private CommandJoystick joystick;
  private Drivetrain drivetrain;
  private Arm arm;
  // private Claw claw;
  // private Vision vision;
  private MecanumAutoBuilder autoBuilder;
  private PathConstraints pathConstraints;
  private PathPlannerUtil pathPlannerUtil;
  private SelectCommand selectCommand;
  private SendableChooser<String> pathChooser;
  List<PathPlannerTrajectory> pathGroup;

  public RobotContainer(BiConsumer<Runnable, Double> periodic) {
    drivetrain = new Drivetrain();
    // claw = new Claw();
    arm = new Arm();
    // vision = new Vision(drivetrain::getPose, drivetrain::updatePose);
    // periodic.accept(vision::update, 0.1); // setup the vision system to update every .1 seconds

    setupPathPlanner();

    xboxController = new NWUXboxController(0);
    joystick = new CommandJoystick(1);
    configureBindings();

    
  }

  private void configureBindings() {
    // xboxController.a().onTrue(claw.openCommand());
    // xboxController.b().onTrue(claw.closeConeCommand());
    // xboxController.x().onTrue(claw.closeCubeCommand());

    drivetrain.setDefaultCommand(drivetrain.defaultDriveCommand(
      xboxController::getLeftX,
      xboxController::getLeftY, 
      xboxController::getRightX));

    xboxController.rightBumper().onTrue(drivetrain.lowerMultiplier());
    xboxController.leftBumper().onTrue(drivetrain.raiseMultiplier());
    
    xboxController.start().onTrue(drivetrain.toggleFieldOriented());
    xboxController.back().onTrue(drivetrain.resetGyro());
    
    joystick.button(2).onTrue(arm.stop());
    joystick.button(1).whileTrue(arm.manualPositionCommand(joystick::getX, joystick::getTwist));

    joystick.button(11).onTrue(arm.setPositionCommand(ArmPosition.TRAVELING));
    joystick.button(12).onTrue(arm.setPositionCommand(ArmPosition.LOADING));
    joystick.button(8).onTrue(arm.setPositionCommand(ArmPosition.CONE_2));
  }

  private void setupPathPlanner() {
    
    pathConstraints = new PathConstraints(3, 4);
    pathGroup = PathPlanner.loadPathGroup("Straight", pathConstraints);
    
    Map<String, Command> eventMap = Map.of(
      "DropCone", new PrintCommand("Drop Cone!"),
      "DropCube", new PrintCommand("Drop Cube!"),
      "PickupCone", new PrintCommand("Pickup Cone!"),
      "PickupCube", new PrintCommand("Pickup Cube!"),
      "ChargeStation", new PrintCommand("Charge Station!")
    );

    autoBuilder = new MecanumAutoBuilder(
      drivetrain::getPose,
      drivetrain::resetPose,
      Constants.DriveConstants.DRIVE_KINEMATICS,
      new PIDConstants(2.9, 0, 0), // 048199 we will use the smart controllers
      new PIDConstants(2, 0, 0),
      Constants.DriveConstants.MAX_SPEED,
      drivetrain::outputWheelSpeeds,
      eventMap,
      true,
      drivetrain); //, arm, claw

    // pathChooser.addOption("Straight", "Straight");

    // var p = Paths.First;
    // Map<String, Command> pathSelectMap = Map.of(
    //   "Straight", choosePath("Straight")
    // );
    // selectCommand = new SelectCommand(pathSelectMap, null);
  }

  enum Paths {
    Straignt, First, Second
  }

  private Command choosePath(String s) {
    return Commands.runOnce(() -> {

    });
  }

  private Command newPathCommand() {
    return Commands.runOnce(() -> pathPlannerUtil
      .generateTrajectory(pathConstraints, drivetrain.getPose(), drivetrain.getChassisSpeeds())
      .ifPresent(t -> autoBuilder.followPath(t).schedule()), drivetrain); // arm, claw
  }

  // private Command getAutoArmCommand() {
  //   Command moveArm = arm.setPositionCommand(ArmHeight.CONE_1);
  //   Command openClaw = Commands.waitUntil(arm::isAtArmGoal).andThen(claw.openCommand());
  //   return Commands.parallel(
  //     moveArm, openClaw
  //   );
  // }

  private void prepareForTeleop(boolean canceled) {
    // TODO: set to field oriented
    drivetrain.toggleFieldOriented();
    drivetrain.setAutonomousMode(false);
  }
  
  public Command getAutonomousCommand() {
    return drivetrain.resetGyro().
    andThen(
    autoBuilder.resetPose(pathGroup.get(0)).andThen(autoBuilder.followPathGroupWithEvents(pathGroup)).finallyDo(this::prepareForTeleop));
  }
}
