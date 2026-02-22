// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class climbSetup extends SequentialCommandGroup {
  public climbSetup(
      ClimberSubsystem climber) {
    addCommands(
        new ParallelCommandGroup(
            new setAltPosition(climber, ClimberConstants.kClimbHeight),
            new setClawPosition(climber, Value.kForward)),
        new setRollPosition(climber, ClimberConstants.kClimbRoll));
  }
}
