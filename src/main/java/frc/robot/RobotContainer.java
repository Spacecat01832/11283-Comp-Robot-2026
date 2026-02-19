// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  private final AutoSubsystem auto = new AutoSubsystem();

  final CommandXboxController driverXbox = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {

  }

  public Command getAutonomousCommand() {
    return auto.getSelectedAuto();
  }
}