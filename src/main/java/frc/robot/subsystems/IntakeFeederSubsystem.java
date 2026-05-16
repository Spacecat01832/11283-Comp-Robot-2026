// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;

public class IntakeFeederSubsystem extends SubsystemBase {

  private TalonFX IntakeMotor = new TalonFX(MotorIDs.kIntake);
  private double intakespeed;

  private SparkMax IntakeFlopperMotor = new SparkMax(MotorIDs.kIntakeFlopper, MotorType.kBrushless);
  private boolean floppersetpoint;
  private Timer flopperTimer = new Timer();

  private SparkMax feederMotor = new SparkMax(MotorIDs.kFeeder, MotorType.kBrushless);
  private double feedspeed;

  private SparkMax indexMotor = new SparkMax(MotorIDs.kIndexer, MotorType.kBrushless);
  private double indexspeed;

  private SparkClosedLoopController flopperPID = IntakeFlopperMotor.getClosedLoopController();

  public IntakeFeederSubsystem() {
    flopperTimer.start();
    floppersetpoint = false;
    feedspeed = 0;
    indexspeed = 0;
    intakespeed = 0;
  }

  @Override
  public void periodic() {
    IntakeFlopperMotor.setVoltage(cuspid(
        IntakeFlopperMotor.getEncoder().getPosition(),
        floppersetpoint,
        flopperTimer.get()));
    IntakeMotor.set(intakespeed);
    feederMotor.set(feedspeed);
    indexMotor.set(indexspeed);
  }

  private double cuspid(double current, boolean out, double timer_value) {
    double voltage = 0.0;

    var holdkick = 12;
    var holdcruse = 2.5;
    var outkick = -3.0;
    var outcruse = -1.6;
    var kicktime = 0.2;
    if (out) {
      voltage = timer_value < kicktime ? outkick : outcruse;
    } else {
      voltage = timer_value < kicktime ? holdkick : holdcruse;
    }

    var target = out ? IntakeConstants.koutPosition : 0.0;
    var error = target - current;
    var abs_error = Math.abs(error);
    var error_range = 0.4;
    if (abs_error < error_range) {
      double scale = MathUtil.clamp(abs_error / error_range, 0.2, out ? outcruse : holdcruse);
      voltage *= scale;
    }

    var tolerance = 0.1;
    if (out
        ? error > -tolerance
        : error < tolerance) {
      voltage = 0;
    }

    return voltage;
  }

  public void setFeeder(double speed) {
    feedspeed = speed;
  }

  public void setIntake(double speed) {
    intakespeed = speed;
  }

  public void setIndexer(double speed) {
    indexspeed = speed;
  }

  public void setIntakeFlopper(boolean out) {
    flopperTimer.reset();
    floppersetpoint = out;
  }

  public double getIntakeFlopperPosition() {
    return IntakeFlopperMotor.getEncoder().getPosition();
  }

  public boolean isIntakeFlopperAtGoal() {
    return flopperPID.isAtSetpoint();
  }
}
