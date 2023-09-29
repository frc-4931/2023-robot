package frc.robot.util;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class NWUXboxController extends CommandXboxController {

  public NWUXboxController(int port) {
    super(port);
  }

  @Override
  public double getLeftX() {
    return -super.getLeftY();
  }

  @Override
  public double getLeftY() {
    return super.getLeftX();
  }
  
  // @Override
  // public double getRightX() {
  //   return -super.getRightX();
  // }

  // @Override
  // public double getRightY() {
  //   return -super.getRightX();
  // }

}
