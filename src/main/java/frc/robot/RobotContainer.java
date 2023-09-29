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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import frc.robot.subsystems.Drivetrain2;
import frc.robot.subsystems.Vision;
import frc.robot.subsystems.Arm.ArmPosition;
import frc.robot.util.NWUXboxController;
import frc.robot.util.PathPlannerUtil;

public class RobotContainer {
  private CommandXboxController xboxController;
  private CommandJoystick joystick;
  private Drivetrain2 drivetrain2;
  // private Arm arm;
  private Claw claw;
  // private Vision vision;
  private MecanumAutoBuilder autoBuilder;
  private PathConstraints pathConstraints;
  private PathPlannerUtil pathPlannerUtil;
  private SelectCommand selectCommand;
  private SendableChooser<String> pathChooser;
  List<PathPlannerTrajectory> pathGroup;
  private UsbCamera camera1, camera2;
  // NetworkTableEntry cameraSelection;
  private MjpegServer switchCam;

  public RobotContainer(BiConsumer<Runnable, Double> periodic) {
    drivetrain2 = new Drivetrain2();
    claw = new Claw();
    // arm = new Arm();
    // vision = new Vision(drivetrain::getPose, drivetrain::updatePose);
    // periodic.accept(vision::update, 0.1); // setup the vision system to update every .1 seconds

    // setupPathPlanner();

    xboxController = new NWUXboxController(0);
    joystick = new CommandJoystick(1);
    configureBindings();

    // arm.setPositionCommand(ArmPosition.UP).schedule();

    camera1 = CameraServer.startAutomaticCapture("FrontCamera", 0);
    camera2 = CameraServer.startAutomaticCapture("BackCamera", 1);
    switchCam = CameraServer.addSwitchedCamera("VideoSource");
    
  }

  private void configureBindings() {
    xboxController.a().onTrue(claw.openCommand());
    xboxController.b().onTrue(claw.closeConeCommand());
    xboxController.x().onTrue(claw.closeCubeCommand());

    drivetrain2.setDefaultCommand(drivetrain2.defaultDriveCommand(
      xboxController::getLeftX,
      xboxController::getLeftY, 
      xboxController::getRightX));

    xboxController.rightBumper().onTrue(drivetrain2.lowerMultiplier());
    xboxController.leftBumper().onTrue(drivetrain2.raiseMultiplier());
    
    xboxController.start().onTrue(drivetrain2.toggleFieldOriented());
    xboxController.back().onTrue(drivetrain2.resetGyro());
    
    // joystick.button(2).onTrue(arm.stop());
    // joystick.button(5).whileTrue(arm.manualPositionCommand(joystick::getX, joystick::getTwist, true, joystick::getThrottle));
    // joystick.button(3).whileTrue(arm.manualPositionCommand(joystick::getX, joystick::getTwist, false, joystick::getThrottle));
    
    // joystick.button(11).onTrue(arm.setPositionCommand(ArmPosition.UP));
    // joystick.button(12).onTrue(arm.setPositionCommand(ArmPosition.LOADING));
    // joystick.button(8).onTrue(arm.setPositionCommand(ArmPosition.CONE_2));

    joystick.button(6).onTrue(Commands.runOnce(() -> {
      switchCam.setSource(camera1);
    }));
    joystick.button(4).onTrue(Commands.runOnce(() -> {
      switchCam.setSource(camera2);
    }));
  }

  public Command getAutonomousCommand() {
    return drivetrain2.resetGyro().andThen(claw.openCommand()).
    andThen(Commands.run(() -> {
      drivetrain2.drive(-0.5, 0, 0);
    }, drivetrain2).withTimeout(.5));
    // autoBuilder.resetPose(pathGroup.get(0)).andThen(autoBuilder.followPathGroupWithEvents(pathGroup)).finallyDo(this::prepareForTeleop));
  }
}
