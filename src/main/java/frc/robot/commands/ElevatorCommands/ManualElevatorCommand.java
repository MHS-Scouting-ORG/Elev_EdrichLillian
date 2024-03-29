// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ManualElevatorCommand extends Command {
 private ElevatorSubsystem elevSubsystem;
 private double speed;
  public ManualElevatorCommand(ElevatorSubsystem elevsubs, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    elevSubsystem = elevsubs;
    this.speed = speed;

    addRequirements(elevSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){
    elevSubsystem.ManualElevator(speed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    elevSubsystem.elevStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
