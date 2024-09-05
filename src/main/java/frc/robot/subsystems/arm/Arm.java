// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonSRX pivotMotor;

  public Arm() {
    pivotMotor = new TalonSRX(MotorIDs.pivotMotorTalonID);
  }

  public Command movePivot(DoubleSupplier armMovement) {
    return run(
        () -> {
          double pivot = 0;
          pivot = armMovement.getAsDouble();
          pivotMotor.set(ControlMode.PercentOutput, pivot);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
