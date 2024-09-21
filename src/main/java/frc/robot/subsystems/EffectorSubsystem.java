// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.EffectorConstants;
import frc.robot.Constants.MotorIDs;

public class EffectorSubsystem extends SubsystemBase {
  private final TalonSRX effectorMotor1;

  private TalonSRX wristMotor;
  private CANcoder wristEncoder;
  private PIDController wristPID;

  /** Creates a new EffectorSubsystem. */
  public EffectorSubsystem() {
    effectorMotor1 = new TalonSRX(MotorIDs.effectorMotor1);
  }

  // Commands
  public Command scoreLowCommand() {
    return new PIDCommand(
        wristPID,
        () -> wristEncoder.getPosition().getValueAsDouble(),
        EffectorConstants.lowScore,
        (output) -> wristMotor.set(ControlMode.PercentOutput, output),
        this);
  }

  public Command scoreHighCommand() {
    return new PIDCommand(
        wristPID,
        () -> wristEncoder.getPosition().getValueAsDouble(),
        EffectorConstants.highScore,
        (output) -> wristMotor.set(ControlMode.PercentOutput, output),
        this);
  }

  public Command intakeCommand() {
    return startEnd(
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, EffectorConstants.intakePercent);
        },
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, 0);
        });
  }

  public Command outakeCommand() {
    return startEnd(
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, EffectorConstants.outtakePercent);
        },
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, 0);
        });
  }

  public Command stopCommand() {
    return run(
        () -> {
          effectorMotor1.set(ControlMode.PercentOutput, 0.0);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
