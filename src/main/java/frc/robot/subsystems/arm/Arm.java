package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {

  private final ArmIO io;

  public Arm(ArmIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {}

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void runVelocity(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocity(velocityRadPerSec);
    Logger.recordOutput("Arm/SetpointRPM", velocityRPM);
  }

  /** Stops the Arm. */
  public void stop() {
    io.stop();
  }
}
