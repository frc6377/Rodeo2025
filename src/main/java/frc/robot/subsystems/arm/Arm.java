// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  public Arm() {
    public Command movePivot(DoubleSupplier armMovement) {
      return run(
        () -> {
          
        }
      );
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
