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
            driveSubsystem.setForwardCommand(2, 0.5),
            // armSubsystem.scoreHighCommand(),
            // effectorSubsystem.outakeCommand(),
            Commands.waitSeconds(1),
            driveSubsystem.setForwardCommand(2, -0.5),
            Commands.waitSeconds(1),
            driveSubsystem.turnCommand(90),
            AutoForwardTest(driveSubsystem))
        .withName("OneBeackerAuto");
  }

  // Commands
  // Note: forwards and backwards are reversed for the drive base due to how the Xbox controller
  // works
  // So negative numbers will be forwards, and positive will be backwards.
  public static Command AutoTurn90Test(DriveTrainSubsystem driveTrainSubsystem) {
    return Commands.sequence(
            driveTrainSubsystem.turnCommand(90),
            Commands.waitSeconds(2),
            driveTrainSubsystem.turnCommand(0))
        .withName("AutoTurn90Test");
  }

  public static Command AutoForwardTest(DriveTrainSubsystem driveTrainSubsystem) {
    return Commands.sequence(
            driveTrainSubsystem.setForwardCommand(0.75, -0.5),
            Commands.waitSeconds(2),
            driveTrainSubsystem.setForwardCommand(0.75, 0.5))
        .withName("AutoForwardTest");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
