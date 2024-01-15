package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorToTopCommand extends Command {

  private ElevatorSubsystem elevSub;

  public ElevatorToTopCommand(ElevatorSubsystem newElevSub) {
    elevSub = newElevSub;
    addRequirements(elevSub);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    elevSub.toTop();
  }

  @Override
  public void end(boolean interrupted) {
    elevSub.elevStop();
  }

  @Override
  public boolean isFinished() {
    return elevSub.getTopLimitSwitch();
  }
}
