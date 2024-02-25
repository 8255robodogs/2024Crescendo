// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto.programs;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.ExampleCmd;
import frc.robot.commands.auto.FollowTrajectoryCmd;
import frc.robot.commands.drivetrain.SetTranslationCmd;
import frc.robot.subsystems.ExampleSys;
import frc.robot.subsystems.SwerveSys;

public class ExampleAuto2 extends SequentialCommandGroup {
  /** Creates a new ExampleAuto. */
  public ExampleAuto2(SwerveSys swerveSys, ExampleSys exampleSys) {
    addCommands(
      new SetTranslationCmd(new Translation2d(0.0, 0.0), swerveSys),
      new FollowTrajectoryCmd("Short Path Forward", 0.1, swerveSys)
      
    );
  }
}
