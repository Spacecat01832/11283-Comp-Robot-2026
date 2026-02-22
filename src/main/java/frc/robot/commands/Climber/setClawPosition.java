// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimberSubsystem;

public class setClawPosition extends InstantCommand {
  private ClimberSubsystem climber;
  Value value = Value.kReverse;

  public setClawPosition(
      ClimberSubsystem climber,
      Value value) {
    addRequirements(climber);
    this.climber = climber;
    this.value = value;
  }

  @Override
  public void initialize() {
    climber.setClaw(value);
  }
}
