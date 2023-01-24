// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ArmExtension;
import frc.robot.subsystems.Claw;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {
  private Drivetrain drivetrain;
  private Arm arm;
  private ArmExtension armExtension;
  private Claw claw;
  private CommandXboxController xboxController;

  public RobotContainer() {
    arm = new Arm();
    armExtension = new ArmExtension();
    claw = new Claw();
    drivetrain = new Drivetrain();

    configureBindings();
  }

  private void configureBindings() {
    
    
    Commands.run(() -> drivetrain.drive(0, 0, 0), drivetrain);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
