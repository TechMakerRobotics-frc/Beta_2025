package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoSimple;
// import frc.robot.subsystems.arm.Arm;
// import frc.robot.subsystems.arm.ArmIOSpark;
import frc.robot.subsystems.cradle.Cradle;
import frc.robot.subsystems.cradle.CradleIOSpark;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {
  private final Drivetrain drive = Drivetrain.getInstance();
  private final Cradle cradle = new Cradle(new CradleIOSpark());
  // private final Arm arm = new Arm(new ArmIOSpark());

  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static final ShuffleboardTab mainTab = Shuffleboard.getTab("Robot");

  private Trigger triggerLeft = new Trigger(controller.leftTrigger(0.07));
  private Trigger triggerRight = new Trigger(controller.rightTrigger(0.07));

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.setDriveMotors(controller.getLeftY(), controller.getRightX() * -1), drive));

    triggerLeft
        .onTrue(new InstantCommand(() -> cradle.runVelocity(controller.getLeftTriggerAxis() * 2)))
        .onFalse(new InstantCommand(() -> cradle.stop()));

    triggerRight
        .onTrue(new InstantCommand(() -> cradle.runVelocity(-controller.getRightTriggerAxis() * 2)))
        .onFalse(new InstantCommand(() -> cradle.stop()));
  }

  public Command getAutonomousCommand() {
    return new AutoSimple();
  }
}
