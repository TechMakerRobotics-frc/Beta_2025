package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Auto.AutoSimple;
import frc.robot.subsystems.drive.Drivetrain;

public class RobotContainer {
  private final Drivetrain drive = Drivetrain.getInstance();

  private final CommandXboxController controller =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  public static final ShuffleboardTab mainTab = Shuffleboard.getTab("Robot");

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    drive.setDefaultCommand(
        new RunCommand(
            () -> drive.setDriveMotors(controller.getLeftY() * -1, controller.getRightX() * -1), drive));
  }

  public Command getAutonomousCommand() {
    return new AutoSimple();
  }
}
