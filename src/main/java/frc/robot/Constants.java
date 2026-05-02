// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.generated.TunerConstants;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class MotorIDs {
    public static final int kShooter = 1;
    public static final int kIndexer = 3;
    public static final int kFeeder = 4;
    public static final int kIntake = 5;
    public static final int kIntakeFlopper = 6;

  }

  public static class ShooterConstants {
    public static InterpolatingDoubleTreeMap kShooterSpeedMap = new InterpolatingDoubleTreeMap();
    static {
      kShooterSpeedMap.put(2.0, 57.0);
      kShooterSpeedMap.put(3.0, 65.0);
      kShooterSpeedMap.put(4.0, 95.0);
    }
  }

  public static class IntakeConstants {
    public static double kIntakeSpeed = 0.75;
    public static double kFeederSpeed = 1;
    public static double kIndexerSpeed = 1;
    public static double koutPosition = -1;
    public static double kIntakePoSpeed = 0.5;
  }

  public static class DriveConstants {
    public static double kMaxSpeed = 0.48 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static double kMaxAngularRate = RotationsPerSecond.of(1.5).in(RadiansPerSecond);
  }
}

/*
 * shooter hood sparkmax conversion is at 110.7692307692
 */