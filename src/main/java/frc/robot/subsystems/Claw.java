package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Claw extends SubsystemBase {
  private Compressor compressor = new Compressor(0, PneumaticsModuleType.CTREPCM);
  private DoubleSolenoid rod1, rod2;

  public Claw() {
<<<<<<< HEAD
    rod1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 5, 3);
    rod2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 4, 2);
=======
    rod1 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 4);
    rod2 = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 5);
>>>>>>> 21ca8a442776a2b256a067fa7b3d27701e6f5098
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
