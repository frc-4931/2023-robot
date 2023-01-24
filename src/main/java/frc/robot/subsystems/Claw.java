package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private DoubleSolenoid rod1;
  private DoubleSolenoid rod2;

  public Claw() {
    rod1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
    rod2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 0, 0);
  }

  public void open() {
    rod1.set(Value.kForward);
    rod2.set(Value.kForward);
  }

  public void closeOnCube() {
    rod1.set(Value.kReverse);
  }

  public void closeOnCone() {
    rod1.set(Value.kReverse);
    rod2.set(Value.kReverse);
  }
}
