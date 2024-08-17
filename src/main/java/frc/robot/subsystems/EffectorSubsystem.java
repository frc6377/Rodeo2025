// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EffectorConstants;
import frc.robot.Constants.MotorIDs;

public class EffectorSubsystem extends SubsystemBase {
  private final TalonSRX effectorMotor1;
  private final TalonSRX effectorMotor2;

  /** Creates a new EffectorSubsystem. */
  public EffectorSubsystem() {
    effectorMotor1 = new TalonSRX(MotorIDs.effectorMotor1);
    effectorMotor2 = new TalonSRX(MotorIDs.effectorMotor2);
  }

  // Commands
  public Command intakeCommand() {
    return run(
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, EffectorConstants.intakePercent);
          effectorMotor2.set(ControlMode.PercentOutput, EffectorConstants.intakePercent);
        });
  }

  public Command outakeCommand() {
    return run(
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, EffectorConstants.outakePercent);
          effectorMotor2.set(ControlMode.PercentOutput, EffectorConstants.outakePercent);
        });
  }

  public Command stopCommand() {
    return run(
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, 0.0);
          effectorMotor2.set(ControlMode.PercentOutput, 0.0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
