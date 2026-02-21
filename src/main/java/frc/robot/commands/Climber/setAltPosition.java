// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class setAltPosition extends InstantCommand {
  private ClimberSubsystem climber;
  double setpoint;
  public setAltPosition(
    ClimberSubsystem climber,
    double setpoint
  ) {
    addRequirements(climber);
    this.climber = climber;
    this.setpoint = setpoint;
  }

  @Override
  public void initialize() {
    climber.setAltGoal(setpoint);
  }
}
