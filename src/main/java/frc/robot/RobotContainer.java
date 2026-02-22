// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.*;
import frc.robot.commands.Climber.climbSetup;
import frc.robot.commands.Climber.setAltPosition;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Auto.AutoSubsystem;

public class RobotContainer {

  final AutoSubsystem auto = new AutoSubsystem();

  final TurretSubsystem turret = new TurretSubsystem();
  final IntakeFeederSubsystem intakeFeeder = new IntakeFeederSubsystem();
  final ClimberSubsystem climber = new ClimberSubsystem();

  final climbSetup climbSetup = new climbSetup(climber);

  final CommandXboxController driverController = new CommandXboxController(OperatorConstants.kDriverControllerPort);
  final CommandXboxController secondDriverController = new CommandXboxController(
      OperatorConstants.k2ndDriverControllerPort);

  public RobotContainer() {
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
  }

  private void configureBindings() {
    secondDriverController.y()
        .whileTrue(climbSetup)
        .whileFalse(new setAltPosition(climber, 0));
  }

  public Command getAutonomousCommand() {
    return auto.getSelectedAuto();
  }
}