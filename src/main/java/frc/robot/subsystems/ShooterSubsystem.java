// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax shooterMain =
      new SparkMax(RobotConstants.kShooterMainCanId, MotorType.kBrushless);
  private SparkMax shooterSecond =
      new SparkMax(RobotConstants.kShooterSecondaryCanId, MotorType.kBrushless);

  public ShooterSubsystem() {}

  public void shooterForwardCmd() {
    shooterMain.set(0.8);
    shooterSecond.set(-0.8);
  }

  public void shooterReverseCmd() {
    shooterMain.set(-0.8);
    shooterSecond.set(0.8);
  }

  public void shooterStopCmd() {
    shooterMain.stopMotor();
    shooterSecond.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
