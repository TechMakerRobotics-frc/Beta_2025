package frc.robot.subsystems.elevator;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void set(double power) {
    io.set(power);
    Logger.recordOutput("Elevator/SetpointRPM", power);
  }

  /** Stops the Elevator. */
  public void stop() {
    io.stop();
  }
}
