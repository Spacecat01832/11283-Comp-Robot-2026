// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Auto;

import java.net.JarURLConnection;
import java.net.URL;
import java.nio.file.DirectoryStream;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.Enumeration;
import java.util.jar.JarEntry;
import java.util.jar.JarFile;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AutoSubsystem extends SubsystemBase {
  public final SendableChooser<Command> AutoChooser = new SendableChooser<>();

  private static final String AUTO_PACKAGE = "frc.robot.commands.auto";

  public AutoSubsystem() {
    // Make sure there's always a "None" option
    AutoChooser.setDefaultOption("None", null);
    // Discover and add auto commands from the package
    discoverAndAddAutos(AUTO_PACKAGE);
    // putting it on smartdashboard
    SmartDashboard.putData("AutoChooser", AutoChooser);
  }

  /**
   * Discover classes in the given package, instantiate those that implement
   * Command
   * and add them to the chooser. This handles both file-system and JAR-based
   * classpaths.
   * Requires the auto command classes to have a public no-arg constructor.
   */
  private void discoverAndAddAutos(String packageName) {
    String path = packageName.replace('.', '/');
    try {
      Enumeration<URL> resources = Thread.currentThread().getContextClassLoader().getResources(path);
      while (resources.hasMoreElements()) {
        URL resource = resources.nextElement();
        String protocol = resource.getProtocol();

        if ("file".equals(protocol)) {
          // Running from classes in the file system (likely during development /
          // simulation)
          Path dir = Paths.get(resource.toURI());
          try (DirectoryStream<Path> stream = Files.newDirectoryStream(dir, "*.class")) {
            for (Path p : stream) {
              String fileName = p.getFileName().toString();
              String className = fileName.substring(0, fileName.length() - 6); // strip .class
              String fqcn = packageName + "." + className;
              tryCreateAndAdd(fqcn, className);
            }
          }
        } else if ("jar".equals(protocol)) {
          // Running from a JAR (packaged)
          JarURLConnection jarConn = (JarURLConnection) resource.openConnection();
          try (JarFile jar = jarConn.getJarFile()) {
            Enumeration<JarEntry> entries = jar.entries();
            while (entries.hasMoreElements()) {
              JarEntry entry = entries.nextElement();
              String name = entry.getName();
              if (name.startsWith(path) && name.endsWith(".class") && !entry.isDirectory()) {
                String fqcn = name.replace('/', '.').substring(0, name.length() - 6);
                String simpleName = fqcn.substring(fqcn.lastIndexOf('.') + 1);
                tryCreateAndAdd(fqcn, simpleName);
              }
            }
          }
        }
      }
    } catch (Exception e) {
      // Keep chooser usable even if discovery fails
      System.err.println("Auto discovery failed: " + e.getMessage());
      e.printStackTrace();
    }
  }

  private void tryCreateAndAdd(String fqcn, String displayName) {
    try {
      Class<?> cls = Class.forName(fqcn);
      if (Command.class.isAssignableFrom(cls)) {
        // Attempt to instantiate via no-arg constructor
        try {
          Command cmd = (Command) cls.getDeclaredConstructor().newInstance();
          addAuto(displayName, cmd);
        } catch (NoSuchMethodException nsme) {
          System.err.println("Auto class " + fqcn + " has no public no-arg constructor; skipping.");
        }
      }
    } catch (ClassNotFoundException cnfe) {
      System.err.println("Auto class not found: " + fqcn);
    } catch (Exception ex) {
      System.err.println("Failed to instantiate auto " + fqcn + ": " + ex.getMessage());
      ex.printStackTrace();
    }
  }

  public void addAuto(String name, Command command) {
    if (command == null) {
      // skip null command additions (we already added a None default)
      return;
    }
    AutoChooser.addOption(name, command);
  }

  public Command getSelectedAuto() {
    return AutoChooser.getSelected();
  }
}
