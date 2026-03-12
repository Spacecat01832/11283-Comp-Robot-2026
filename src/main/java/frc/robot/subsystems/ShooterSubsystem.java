// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.ShooterConstants;

public class ShooterSubsystem extends SubsystemBase {

  private final TalonFX shooterMotor = new TalonFX(MotorIDs.kShooter);
  private final SparkMax hoodMotor = new SparkMax(MotorIDs.kShooterHood, MotorType.kBrushed);
  private SparkClosedLoopController hoodPid = hoodMotor.getClosedLoopController();

  private PIDController ShooterLimiter = new PIDController(
      0.008,
      0.0005,
      0.0001);

  private SimpleMotorFeedforward shooterFeed = new SimpleMotorFeedforward(0.01, 0.0098);

  

  public ShooterSubsystem() {
    ShooterLimiter.setSetpoint(0);
  }

  @Override
  public void periodic() {
    shooterMotor.set(
        ShooterLimiter.calculate(shooterMotor.getVelocity().getValueAsDouble())
            + shooterFeed.calculate(shooterMotor.getVelocity().getValueAsDouble()));
  }

  public void setHoodGoal(double angle) {
    hoodPid.setSetpoint(angle < 1.5
        ? 1.5
        : angle > ShooterConstants.kHoodMaxAngle
            ? ShooterConstants.kHoodMaxAngle
            : angle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }

  public boolean atHoodGoal() {
    return hoodPid.isAtSetpoint();
  }

  public double getHoodPosition() {
    return hoodMotor.getEncoder().getPosition();
  }

  public void setShooterSpeed(double speed) {
    ShooterLimiter.setSetpoint(speed);
  }
}
