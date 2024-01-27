package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.ElevatorCommands.ElevatorSetpointCmd;
import frc.robot.commands.ElevatorCommands.ManualElevatorCommand;
import frc.robot.subsystems.ElevatorSubsystem;

public class RobotContainer {
  private final ElevatorSubsystem elevSub = new ElevatorSubsystem();

  private final XboxController xboxController = new XboxController(0);
  private final Joystick joystick = new Joystick(1);

  public RobotContainer() {
    //elevSub.setDefaultCommand(new Elevator(elevSub, () -> joystick.getY()));
    elevSub.setDefaultCommand(new ManualElevatorCommand(elevSub, joystick.getY()));

    configureBindings();
  }

  private void configureBindings() {
    //new JoystickButton(joystick,   6).onTrue(new ElevatorToTopCommand(elevSub));
    //new JoystickButton(joystick, 4).onTrue(new ElevatorToBottomCommand(elevSub));

    //new JoystickButton(joystick, 2).onTrue(new ElevatorSetpointCmd(elevSub, /* */ )); *COMMAND TO RUN ELEVATOR AT SETPOINT
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
