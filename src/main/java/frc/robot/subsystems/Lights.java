package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {
  private static final int[] BLUE = new int[] { 206 / 2, 1 * 255, Math.round(0.70f * 255) };
  private static final int[] RED = new int[] { 358 / 2, Math.round(0.85f * 255), Math.round(0.93f * 255) };
  
  private AddressableLED led;
  private AddressableLEDBuffer ledBuffer;
  // private boolean on = false;
  
  public Lights(boolean isRed) {
    
    led = new AddressableLED(9); // PWM
    ledBuffer = new AddressableLEDBuffer(160);
    led.setLength(ledBuffer.getLength());

    led.setData(ledBuffer);
    // led.start();
    
  }

  // public void setOn(boolean on) {
  //   this.on = on;
  //   if (on) {
  //     led.start();
  //   }
  //   else {
  //     ledBuffer.setHSV(0, 0, 0, 0);
  //     led.stop();
  //   }
  // }

  private void setBlack(boolean interrupted) {
    ledBuffer.setHSV(0, 0, 0, 0);
    led.stop();
  }

  // public Command runBubbles() {

  // }
  private class BubblesCommand extends CommandBase {
    private int[][] leftPos = new int[2][3];
    private int[] left = new int[] { 91, 140, 148 };
    private int[][] rightPos = new int[2][3];
    private int[] right = new int[] { 84, 140, 148 };
    private Timer timer = new Timer();

    @Override
    public void initialize() {
      timer.reset();
      timer.start();
    }

    @Override
    public void execute() {
      if (timer.advanceIfElapsed(5)) {


      }
    }
  }

  private class RainbowCommand extends CommandBase {
    RainbowCommand(Lights l) {
      addRequirements(l);
    }

    int m_rainbowFirstPixelHue = 0;
    @Override
    public void initialize() {
      m_rainbowFirstPixelHue = 0;
    }
    @Override
    public void execute() {
      // For every pixel
      for (var i = 0; i < ledBuffer.getLength(); i++) {
        // Calculate the hue - hue is easier for rainbows because the color
        // shape is a circle so only one value needs to precess
        final var hue = (m_rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
        // Set the value
        ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      m_rainbowFirstPixelHue += 3;
      // Check bounds
      m_rainbowFirstPixelHue %= 180;
    }
  }

  public Command runRainbow() {
    return new RainbowCommand(this);
  }

  public Command runTeamChase() {
    CommandBase c = new CommandBase() {
      int sectionLength = 5;
      int offset = 0;
      int[] stamp = new int[6*5];
      int[] color;

      @Override
      public void execute() {
        int stampStart = stamp.length - offset;
        for (int i = 0; i < offset; i++) {
          ledBuffer.setHSV(i, color[0], color[1], stamp[i + stampStart]);
        }
        for (int i = offset; i < ledBuffer.getLength(); i++) {
          int p = i % stamp.length;
          ledBuffer.setHSV(i, color[0], color[1], stamp[p]);
        }
      }

      @Override
      public void initialize() {
        this.color = switch (DriverStation.getAlliance()) {
          case Blue -> BLUE;
          default -> RED;
        };
        
        for (int i = 0; i < sectionLength; i++) {
          stamp[i] = stamp[3*sectionLength - i] = (i + 1) * color[2] / sectionLength;
        }
        
        led.start();
      }

      @Override
      public void end(boolean interrupted) {
        setBlack(interrupted);
      }
    };
    c.addRequirements(this);
    return c;
  }
}
