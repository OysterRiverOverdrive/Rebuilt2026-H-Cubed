// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoIntakeForwardCmd;
import frc.robot.subsystems.IntakeWheelSubsystem;

public class AutoAllianceZonePlan extends ParallelCommandGroup {
  // subsystems
  private final IntakeWheelSubsystem intakewheel = new IntakeWheelSubsystem();

  public AutoAllianceZonePlan() {
    Command spinIntake = // spin the wheel kronk
      new AutoIntakeForwardCmd(intakewheel, 5); 

    addCommands(
      spinIntake

    );
  }
}
