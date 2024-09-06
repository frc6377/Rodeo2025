// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;

public final class Autos {
  /** Example static factory for an autonomous command. */
  public static Command withBeakerCommand(
    Drivetrain drivetrain,
    Arm arm
  ) {
    return Commands.sequence(drivetrain.goForwardCommand(5,2))
    .withName("withBeakerCommand");
  }

  private Autos() {
    throw new UnsupportedOperationException("This is a utility class!");
  }
}
