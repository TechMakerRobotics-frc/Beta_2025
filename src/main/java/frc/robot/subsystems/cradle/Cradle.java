package frc.robot.subsystems.cradle;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Cradle extends SubsystemBase {

  private final CradleIO io;

  public Cradle(CradleIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {}

  public void runVolts(double volts) {
    io.setVoltageForAll(volts);
  }

  public void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityForAll(velocityRadPerSec);
    Logger.recordOutput("Cradle/SetpointRPM", velocityRPM);
  }

  /** Stops the Cradle. */
  public void stop() {
    io.stopAll();
  }
}
