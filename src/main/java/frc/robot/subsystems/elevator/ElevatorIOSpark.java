package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.math.util.Units;
import frc.robot.interfaces.motor.MotorIO;
import frc.robot.interfaces.motor.MotorIO.MotorIOInputs;
import frc.robot.interfaces.motor.MotorIOSparkMax;

public class ElevatorIOSpark implements ElevatorIO {

  private final MotorIO motorLeft;
  private final MotorIO motorRight;

  public ElevatorIOSpark() {
    motorLeft =
        new MotorIOSparkMax(
            ElevatorConstants.MOTOR_LEFT_ID,
            MotorType.kBrushless,
            false,
            250,
            10.0,
            30,
            IdleMode.kBrake);
    motorRight =
        new MotorIOSparkMax(
            ElevatorConstants.MOTOR_RIGHT_ID,
            MotorType.kBrushless,
            true,
            250,
            10.0,
            30,
            IdleMode.kBrake);
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    MotorIOInputs motorIOInputs = motorLeft.getMotorIOInputs();
    inputs.appliedVoltsLeft = motorIOInputs.appliedVolts;
    inputs.currentAmpsLeft = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSecLeft = motorIOInputs.velocityRadPerSec;
    inputs.positionRadLeft = Units.rotationsToRadians(motorIOInputs.positionRot);
    motorIOInputs = motorRight.getMotorIOInputs();
    inputs.appliedVoltsRight = motorIOInputs.appliedVolts;
    inputs.currentAmpsRight = motorIOInputs.currentAmps[0];
    inputs.velocityRadPerSecRight = motorIOInputs.velocityRadPerSec;
    inputs.positionRadRight = Units.rotationsToRadians(motorIOInputs.positionRot);
  }

  @Override
  public void setVoltage(double volts) {
    motorLeft.setVoltage(volts);
    motorRight.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
    motorLeft.setVelocity(velocityRadPerSec * ElevatorConstants.MOTOR_LEFT_GEAR_RATIO);
    motorRight.setVelocity(velocityRadPerSec * ElevatorConstants.MOTOR_RIGHT_GEAR_RATIO);
  }

  @Override
  public void stop() {
    motorLeft.stop();
    motorRight.stop();
  }
}
