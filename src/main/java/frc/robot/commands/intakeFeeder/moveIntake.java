// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeFeeder;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeFeederSubsystem;

public class moveIntake extends InstantCommand {
  IntakeFeederSubsystem intake;
  boolean value;
  public moveIntake(
    IntakeFeederSubsystem intake
  ) {
    addRequirements(intake);
    this.intake = intake;
  }

  @Override
  public void initialize() {
    value = !value;
    if(value){
      intake.setIntakeSolenoid(Value.kForward);
    } else {
      intake.setIntakeSolenoid(Value.kReverse);
    }
    
  }
}
