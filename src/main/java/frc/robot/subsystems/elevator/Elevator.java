package frc.robot.subsystems.elevator;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;

  public Elevator(ElevatorIO io) {
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
    Logger.recordOutput("Elevator/SetpointRPM", velocityRPM);
  }

  /** Stops the Elevator. */
  public void stop() {
    io.stop();
  }
}
