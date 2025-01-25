package frc.robot.subsystems.intake;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

  private final IntakeIO io;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {}

  public void runVoltsIntakeMotor(double volts) {
    io.setVoltageRollerMotor(volts);
  }

  public void runVoltsRollerMotor(double volts) {
    io.setVoltageRollerMotor(volts);
  }

  public void runVelocityIntakeMotor(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityIntakeMotor(velocityRadPerSec);
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  public void runVelocityRollerMotor(double velocityRPM) {
    double velocityRadPerSec = Units.rotationsPerMinuteToRadiansPerSecond(velocityRPM);
    io.setVelocityRollerMotor(velocityRadPerSec);
    Logger.recordOutput("Intake/SetpointRPM", velocityRPM);
  }

  public void stopAll() {
    io.stopAll();
  }

  public void stopIntakeMotor() {
    io.stopIntakeMotor();
  }

  public void stopRollerMotor() {
    io.stopIntakeMotor();
  }
}
