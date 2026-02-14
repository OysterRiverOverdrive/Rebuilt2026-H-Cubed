// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.auto.plans;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.auto.AutoCreationCmd;
import frc.robot.auto.AutoIntakeForwardCmd;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.IntakeWheelSubsystem;
import java.util.List;

public class AutoMiddleFieldPlan extends ParallelCommandGroup {
  // subsystems
  private final IntakeWheelSubsystem intakewheel = new IntakeWheelSubsystem();

  public AutoMiddleFieldPlan(DrivetrainSubsystem drive) {
    Command toMid = 
      AutoCreationCmd.AutoRobotDriveCmd( // start from top threshhold, go to middle, turn to the right along the way
        drive,
        List.of(new Translation2d(4.125, 0)),
        new Pose2d(8.25, 8.0, new Rotation2d(-Math.PI / 2)));
    Command moveDown = 
      AutoCreationCmd.AutoRobotDriveCmd( // move down to where the balls are
        drive, 
        List.of(new Translation2d(0, -3.0)), 
        new Pose2d(8.25, 5.0, new Rotation2d(0)));
    Command spinIntakeWheel = 
      new AutoIntakeForwardCmd(intakewheel, 3); // spins the intake wheel, should run as the robot is moving
    addCommands(
      toMid
        .andThen(moveDown.alongWith(spinIntakeWheel))
    );
  }
}
