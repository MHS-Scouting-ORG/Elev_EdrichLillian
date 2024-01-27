// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;

public class ElevatorSetpointCmd extends Command {
  private ElevatorSubsystem elevSubsystem;
  private double setpoint;
  
  public ElevatorSetpointCmd(ElevatorSubsystem elevSub, double setpoint) {
    elevSubsystem = elevSub;
    this.setpoint = setpoint;
    addRequirements(elevSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize(){

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    elevSubsystem.setpointTo(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
   elevSubsystem.elevStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevSubsystem.atSetpoint();
  }
}
