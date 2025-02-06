package frc.robot.subsystems.elevator;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
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
            false,
            250,
            10.0,
            30,
            IdleMode.kBrake);
    
    motorLeft.setFollower(ElevatorConstants.MOTOR_RIGHT_ID);
    motorRight.setTypeEncoder(FeedbackSensor.kAlternateOrExternalEncoder, true);
    motorRight.setOffset(motorRight.getMotorIOInputs().positionAlternateEncoder);
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
    inputs.positionAlternateEncoder = motorIOInputs.positionAlternateEncoder;
  }

  @Override
  public void setVoltage(double volts) {
    motorRight.setVoltage(volts);
  }

  @Override
  public void setVelocity(double velocityRadPerSec) {
  
    motorRight.setVelocity(velocityRadPerSec);
  }
  @Override
  public void set(double power){
    motorRight.set(power);
  }

  @Override
  public void stop() {
    motorRight.stop();
  }

  @Override
  public void runPosition(double position) {
    motorRight.setPosition(position);
  }
}
