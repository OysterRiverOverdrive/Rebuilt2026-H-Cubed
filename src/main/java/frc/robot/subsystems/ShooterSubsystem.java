// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RobotConstants;

public class ShooterSubsystem extends SubsystemBase {

  private SparkMax shooterMain =
      new SparkMax(RobotConstants.kShooterMainCanId, MotorType.kBrushless);

  private RelativeEncoder shooterEncoder = shooterMain.getEncoder();

  private SparkClosedLoopController shooterPID;

  private SparkMaxConfig shooterConfig = new SparkMaxConfig();

  private double mainSpeed = 12000;

  private double kp = 0.0001;
  private double ki = 0;
  private double kd = 0;

  public ShooterSubsystem() {
    SmartDashboard.putNumber("Main Speed", mainSpeed);
    SmartDashboard.putNumber("shoot p", kp);
    SmartDashboard.putNumber("shoot i", ki);
    SmartDashboard.putNumber("shoot d", kd);

    shooterConfig.idleMode(IdleMode.kCoast)
    .closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(kp, ki, kd);
    
    shooterMain.configure(shooterConfig,
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

    shooterPID = shooterMain.getClosedLoopController();
  }

  public void shooterDislodgeCmd() {
    shooterMain.set(0.2);
  }

  public void shooterShootCmd() {
    shooterPID.setSetpoint(-1 * getMainSpeed(), ControlType.kVelocity);
  }

  public void shooterStopCmd() {
    shooterPID.setSetpoint(0, ControlType.kVelocity);
    shooterMain.stopMotor();
  }

  public double getMainSpeed() {
    return mainSpeed;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter Speed", shooterEncoder.getVelocity());
    // This method will be called once per scheduler run
  }
}
