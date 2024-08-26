// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EffectorSubsystem;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command OneBeackerAuto(
      DriveTrainSubsystem driveSubsystem,
      ArmSubsystem armSubsystem,
      EffectorSubsystem effectorSubsystem) {
    return Commands.sequence(
            driveSubsystem.setForwardCommand(3, 0.5),
            armSubsystem.scoreHighCommand(),
            effectorSubsystem.outakeCommand(),
            driveSubsystem.setForwardCommand(3, -0.5))
        .withName("OneBeackerAuto");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
