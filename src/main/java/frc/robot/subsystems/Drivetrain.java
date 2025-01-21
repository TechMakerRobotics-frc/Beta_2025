package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;

public class Drivetrain extends SubsystemBase {
  private static Drivetrain instance;

  private WPI_VictorSPX motorLeftFront = new WPI_VictorSPX(DrivetrainConstants.kMotorLeftFront);
  private WPI_VictorSPX motorLeftFollower = new WPI_VictorSPX(DrivetrainConstants.kMotorLeftRear);
  private WPI_VictorSPX motorRightFront = new WPI_VictorSPX(DrivetrainConstants.kMotorRightFront);
  private WPI_VictorSPX motorRightFollower = new WPI_VictorSPX(DrivetrainConstants.kMotorRightRear);
  private DifferentialDrive m_diffDrive;

  private DifferentialDriveKinematics kinematics;
  private Field2d field = new Field2d();

  private DifferentialDrivePoseEstimator m_poseEstimator;

  public Drivetrain() {
    motorRightFront.configFactoryDefault();
    motorRightFollower.configFactoryDefault();
    
    motorLeftFront.configFactoryDefault();
    motorRightFollower.configFactoryDefault();

    motorLeftFollower.setNeutralMode(NeutralMode.Brake);
    motorLeftFront.setNeutralMode(NeutralMode.Brake);
    
    motorRightFollower.setNeutralMode(NeutralMode.Brake);
    motorRightFront.setNeutralMode(NeutralMode.Brake);

    motorRightFollower.follow(motorRightFront);
    motorLeftFollower.follow(motorLeftFront);

    motorRightFront.setInverted(true);
    motorLeftFront.setInverted(false);

    motorRightFollower.setInverted(InvertType.FollowMaster);
    motorLeftFollower.setInverted(InvertType.FollowMaster);
    m_diffDrive = new DifferentialDrive(motorLeftFront, motorRightFront);
  }

  public static Drivetrain getInstance() {
    if (instance == null) {
      instance = new Drivetrain();
    }
    return instance;
  }

  public void setSpeeds(DifferentialDriveWheelSpeeds speeds) {
  }

  public void drive(double xSpeed, double rot) {
    var wheelSpeeds = kinematics.toWheelSpeeds(new ChassisSpeeds(xSpeed, 0.0, rot));
    setSpeeds(wheelSpeeds);
  }

  public void updatePose() {
    field.setRobotPose(m_poseEstimator.getEstimatedPosition());
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Train", m_diffDrive);
  }

  public void feed() {
    m_diffDrive.feed();
  }

  public void setDriveMotors(double forward, double rotation) {
    final var xSpeed = forward;

    final var rot = rotation;

    SmartDashboard.putNumber("Potencia Frente (%)", xSpeed * 40);
    SmartDashboard.putNumber("Potencia Curva (%)", rot * 40);

    double left = xSpeed - rot;
    double right = xSpeed + rot;
    tankDrive(left, right);
  }

  public void setMaxOutput(boolean set) {
    if (set) m_diffDrive.setMaxOutput(DrivetrainConstants.kMaxSpeedArmExtended);
    else m_diffDrive.setMaxOutput(1.0);
  }

  public void arcadeDrive(double forward, double rotation) {
    SmartDashboard.putNumber("Potencia Frente (%)", forward * 40.0);
    SmartDashboard.putNumber("Potencia Curva (%)", rotation * 40.0);
    m_diffDrive.arcadeDrive(forward, rotation);
    m_diffDrive.feed();
  }

  public void tankDriveVolts(double left, double right) {
    motorLeftFront.setVoltage(left);
    motorRightFront.setVoltage(right);
    m_diffDrive.feed();
  }

  public void tankDrive(double left, double right) {
    motorLeftFront.set(left);
    motorRightFront.set(right);
    m_diffDrive.feed();
  }

  public void stopDrivetrain() {
    tankDriveVolts(0, 0);
  }

  public double getLeftPower() {
    return (motorLeftFront.get() + motorLeftFollower.get()) / 2;
  }

  public double getRightPower() {
    return (motorRightFront.get() + motorRightFollower.get()) / 2;
  }
}
