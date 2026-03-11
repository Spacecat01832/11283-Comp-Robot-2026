// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(MotorIDs.kShooter);
  private final SparkMax hoodMotor = new SparkMax(MotorIDs.kShooterHood, MotorType.kBrushless);

  private ProfiledPIDController hoodPid = new ProfiledPIDController(
      0,
      0,
      0,
      new TrapezoidProfile.Constraints(ShooterConstants.kHoodMaxSpeed, 0.1));

  private ElevatorFeedforward hoodFeed = new ElevatorFeedforward(
      0,
      0,
      0);

  private PIDController ShooterLimiter = new PIDController(
      0,
      0,
      0);

  private SimpleMotorFeedforward shooterFeed = new SimpleMotorFeedforward(0, 0);

  private NetworkTable nt = NetworkTableInstance.getDefault().getTable("Shooter");

  private NetworkTableEntry hoodPositionEntry = nt.getEntry("Hood Position"),
      hoodGoalEntry = nt.getEntry("Hood Goal"),
      hoodkp = nt.getEntry("hoodkp"), // TODO doesn't need to be here
      hoodki = nt.getEntry("hoodki"), // TODO doesn't need to be here
      hoodkd = nt.getEntry("hoodkd"), // TODO doesn't need to be here
      hoodkv = nt.getEntry("hoodkv"), // TODO doesn't need to be here
      hoodks = nt.getEntry("hoodks"), // TODO doesn't need to be here
      hoodkg = nt.getEntry("hoodkg"), // TODO doesn't need to be here
      hoodacel = nt.getEntry("hoodacel"), // TODO doesn't need to be here
      hoodmspeed = nt.getEntry("hoodmaxspeed"), // TODO doesn't need to be here
      shooterSpeedEntry = nt.getEntry("Shooter Speed"),
      shooterGoalEntry = nt.getEntry("Shooter Goal"),
      shooterkp = nt.getEntry("shooterkp"), // TODO doesn't need to be here
      shooterki = nt.getEntry("shooterki"), // TODO doesn't need to be here
      shooterkd = nt.getEntry("shooterkd"), // TODO doesn't need to be here
      shooterkv = nt.getEntry("shooterkv"), // TODO doesn't need to be here
      shooterspeed = nt.getEntry("shootersetSpeed"),
      hoodposition = nt.getEntry("hoodposition");

  public ShooterSubsystem() {
    hoodPid.setGoal(0);
    hoodPid.setTolerance(ShooterConstants.kPidDeadband);
    ShooterLimiter.setSetpoint(0);

    hoodkp.setDefaultDouble(0);
    hoodki.setDefaultDouble(0);
    hoodkd.setDefaultDouble(0);
    hoodkv.setDefaultDouble(0);
    hoodks.setDefaultDouble(0);
    hoodkg.setDefaultDouble(0);
    shooterkp.setDefaultDouble(0);
    shooterki.setDefaultDouble(0);
    shooterkd.setDefaultDouble(0);
    shooterkv.setDefaultDouble(0);
    shooterspeed.setDefaultDouble(0);
    hoodposition.setDefaultDouble(0);
    hoodacel.setDefaultDouble(0);
    hoodmspeed.setDefaultDouble(0);
  }

  @Override
  public void periodic() {

    hoodPid.setPID(hoodkp.getDouble(0),
        hoodki.getDouble(0),
        hoodkd.getDouble(0));
    hoodFeed.setKs(hoodks.getDouble(0));
    hoodFeed.setKv(hoodkv.getDouble(0));
    hoodFeed.setKg(hoodkg.getDouble(0));
    hoodPid.setConstraints(new TrapezoidProfile.Constraints(hoodmspeed.getDouble(0), hoodacel.getDouble(0)));

    ShooterLimiter.setPID(shooterkp.getDouble(0), shooterki.getDouble(0), shooterkd.getDouble(0));
    shooterFeed.setKv(shooterkv.getDouble(0));

    hoodPositionEntry.setDouble(hoodMotor.getEncoder().getPosition());
    hoodGoalEntry.setDouble(hoodPid.getSetpoint().position);
    shooterGoalEntry.setDouble(ShooterLimiter.getSetpoint());
    shooterSpeedEntry.setDouble(shooterMotor.getVelocity().getValueAsDouble());

    setHoodGoal(hoodposition.getDouble(0));
    Shooter(shooterspeed.getDouble(0));

    hoodMotor.set(
        hoodPid.calculate(hoodMotor.getEncoder().getPosition()) + hoodFeed.calculate(hoodPid.getSetpoint().velocity));
    shooterMotor.set(ShooterLimiter.calculate(shooterMotor.getVelocity().getValueAsDouble())
        + shooterFeed.calculate(shooterMotor.getVelocity().getValueAsDouble()));
  }

  public void setHoodGoal(double angle) {
    if (angle < 0) {
      angle = 0;
    } else if (angle > ShooterConstants.kHoodMaxAngle) {
      angle = ShooterConstants.kHoodMaxAngle;
    }
    hoodPid.setGoal(angle);
  }

  public boolean atHoodGoal() {
    return hoodPid.atGoal();
  }

  public double getHoodPosition() {
    return hoodMotor.getEncoder().getPosition();
  }

  public void Shooter(double speed) {
    ShooterLimiter.setSetpoint(speed);
  }
}
