// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PneumaticConstants {
    public static final PneumaticsModuleType kPneumaticsModuleType = PneumaticsModuleType.REVPH;
    // TODO module ids are subject to change
    public static final int kIntakeID = 0,
        kClimberID1 = 0,
        kClimberID2 = 0;
  }

  public static class MotorIDs { // TODO Change the ID's to the correct ones
    public static final int kBottemTurret = 0,
        kTopTurret = 0,
        kTurretYaw = 0,
        kIntake = 0,
        kClimberRoll = 0,
        kClimberAlt = 0;
  }

  public static class TurretConstants {
    public ProfiledPIDController kPid = new ProfiledPIDController(
        0.2,
        0.005,
        0.1,
        new TrapezoidProfile.Constraints(0.5, 0.1));
    public static double kPidMax = 0,
        kPidMin = 0,
        kConvertion = 3;
  }

  public static class ClimberConstants {
    public ProfiledPIDController kAltPid = new ProfiledPIDController(
        0.2,
        0.005,
        0.1,
        new TrapezoidProfile.Constraints(0.7, 0.3));
    public ElevatorFeedforward kAltEleFeed = new ElevatorFeedforward(
        1.1,
        1.2,
        1.3);
    public ProfiledPIDController kRollPid = new ProfiledPIDController(
      0.2, 
      0.005, 
      0.1, 
      new TrapezoidProfile.Constraints(0.5, 0.2));
    public static double kAltPidMax = 0,
        kAltPidMin = 0,
        kConvertion = 3;
  }
}
/*
 * pid stuff i found on the internet:
 * Robotics/Motion: KP=(0.2-0.8) KI=(0.005-0.05) KD=(0.1-2.0)
 */