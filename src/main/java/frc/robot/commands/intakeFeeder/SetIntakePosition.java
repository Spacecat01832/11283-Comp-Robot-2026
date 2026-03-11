// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intakeFeeder;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.IntakeFeederSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetIntakePosition extends InstantCommand {
  private final IntakeFeederSubsystem intake;
  private final double position;
  public SetIntakePosition(IntakeFeederSubsystem intake, double position) {
    addRequirements(intake);
    this.intake = intake;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setIntakeFlopper(position);
  }
}
