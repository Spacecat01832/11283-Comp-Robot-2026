// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.generated.TunerConstants;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0,
        kButtonControllerPort = 1;
  }

  public static class MotorIDs { // TODO Change the ID's to the correct ones
    public static final int kShooter = 1,
        kShooterHood = 4,
        kIntake = 1,
        kIntakeMover = 20,
        kFeeder = 3,
        kIndexer = 5;
  }

  public static class ShooterConstants {
    public static double kHoodSpeed = 0.5;
  }

  public static class IntakeConstants {
    public static double kIntakeSpeed = -1, kIntakeMoverSpeed = 0.5, kFeederSpeed = 1, koutPosition = 0;
  }

  public static class DriveConstants {
    public static double kMaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
        kMaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
  }
}