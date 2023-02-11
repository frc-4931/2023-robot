package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private DoubleSolenoid rod1, rod2;

  public Claw() {
    rod1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 4);
    rod2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 7, 6);
  }
  
  public Command openCommand() {
    return this.runOnce(() -> {
      rod1.set(Value.kReverse);
      rod2.set(Value.kReverse);
    });
  }

  public Command closeCubeCommand() {
    return this.runOnce(() -> rod1.set(Value.kForward));
  }

  public Command closeConeCommand() {
    return this.runOnce(() -> {
      rod1.set(Value.kForward);
      rod2.set(Value.kForward);
    });
  }
}
