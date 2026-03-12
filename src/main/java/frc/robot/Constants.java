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

  public static class MotorIDs {
    public static final int kShooter = 1,
        kShooterHood = 2,
        kIndexer = 3,
        kFeeder = 4,
        kIntake = 5,
        kIntakeFlopper = 6;

  }

  public static class ShooterConstants {
    public static double kHoodMaxAngle = 45;
  }

  public static class IntakeConstants {
    public static double kIntakeSpeed = -0.75,
        kIntakeFlopperMaxSpeed = 0.5,
        kFeederSpeed = 0.8,
        kIndexerSpeed = 3000,
        koutPosition = -1.7;
  }

  public static class DriveConstants {
    public static double kMaxSpeed = 0.6 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
        kMaxAngularRate = RotationsPerSecond.of(1.25).in(RadiansPerSecond);
  }
}

/*
 * shooter hood sparkmax conversion is at 110.7692307692
 * intake position sparkmax conversion is at 19.125
 */