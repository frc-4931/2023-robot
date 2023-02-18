package frc.robot.util;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class PIDTuner {
  private Map<String, GenericEntry> entries = new HashMap<>();
  private PIDTuner() {
  }

  public static PIDTuner createTunable(String tabName, String name, CANSparkMax motor) {
    PIDTuner tuner = new PIDTuner();
    ShuffleboardLayout layout = Shuffleboard.getTab(tabName)
      .getLayout(name, BuiltInLayouts.kList)
      .withSize(2, 5);
    tuner.entries.put("kP", layout.add("kP", motor.getPIDController().getP()).getEntry());
    tuner.entries.put("kI", layout.add("kI", motor.getPIDController().getI()).getEntry());
    tuner.entries.put("kD", layout.add("kD", motor.getPIDController().getD()).getEntry());
    return tuner;    
  }

  public void update() {
    entries.entrySet().stream().forEach(entry -> {
      var ntVal = entry.getValue().getDouble(0);
      
    });
  }

  // private static class Tunable implements Sendable {

  //   CANSparkMax[] motors;
  //   Tunable(CANSparkMax[] motors) {
  //     this.motors = motors;
  //   }
    
  //   @Override
  //   public void initSendable(SendableBuilder builder) {
  //     builder.
      
  //   }

  // }
}
