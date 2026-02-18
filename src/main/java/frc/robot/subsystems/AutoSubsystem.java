// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;

import com.fasterxml.jackson.databind.ObjectMapper;

import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
  private final SendableChooser<Command> AutoChooser = new SendableChooser<>();
  private ObjectMapper mapper = new ObjectMapper();
  Path folder = Paths.get("src/main/java/frc/robot/commands/auto");

  public AutoSubsystem() {
  }

  public void sellector() throws IOException {
    AutoChooser.setDefaultOption("Choose", null);

    try (DirectoryStream<Path> stream = Files.newDirectoryStream(folder)) {
      for (Path file : stream) {
        Command command = mapper.readValue(file.toFile(), SequentialCommandGroup.class);
        AutoChooser.addOption(file.getFileName().toString(), command);
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
  }

  @Override
  public void periodic() {
    SmartDashboard.putData("Auto", AutoChooser);
  }

  public Command getSelectedAuto() {
    return (Command) AutoChooser.getSelected();
  }
}
