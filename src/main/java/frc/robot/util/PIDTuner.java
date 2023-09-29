package frc.robot.util;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Function;

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
  private Map<String, Tunable> entries = new HashMap<>();
  private ShuffleboardLayout layout;
  private PIDTuner() {
  }

  public static PIDTuner createTunable(String tabName, String name, CANSparkMax motor) {
    PIDTuner tuner = new PIDTuner();
    tuner.layout = Shuffleboard.getTab(tabName)
      .getLayout(name, BuiltInLayouts.kList)
      .withSize(2, 5);
    tuner.addEntry("kP", motor.getPIDController()::getP, motor.getPIDController()::setP);
    tuner.addEntry("kI", motor.getPIDController()::getI, motor.getPIDController()::setI);
    tuner.addEntry("kD", motor.getPIDController()::getD, motor.getPIDController()::setD);
    tuner.addEntry("kFF", motor.getPIDController()::getFF, motor.getPIDController()::setFF);
    // tuner.addEntry("closedLoopError", motor.getPIDController()::getSmartMotionAllowedClosedLoopError, motor.getPIDController()::setSmartMotionAllowedClosedLoopError);
    return tuner;    
  }

  private void addEntry(String key, DoubleSupplier getter, Consumer<Double> setter) {
    GenericEntry entry = layout.add(key, getter.getAsDouble()).getEntry();
    Tunable t = new Tunable(getter, setter, entry);
    entries.put(key, t);
  }

  public void update() {
    entries.entrySet().stream().forEach(tunerEntry -> {
      var val = tunerEntry.getValue().entry.getDouble(0);
      var lastVal = tunerEntry.getValue().getter.getAsDouble();
      if (Double.compare(val, lastVal) != 0) {
        System.out.printf("Updating %s to value %f%n", tunerEntry.getKey(), val);
        tunerEntry.getValue().setter.accept(val);
      }
    });
  }

  /**
   * InnerPIDTuner
   */
  public record Tunable(DoubleSupplier getter, Consumer<Double> setter, GenericEntry entry) {
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
