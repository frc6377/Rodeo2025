// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
  /** Creates a new Drivetrain. */
  private final TalonSRX leftDriveTalon;

  private final VictorSPX leftDriveVictor;
  private final TalonSRX rightDriveTalon;
  private final VictorSPX rightDriveVictor;

  public Drivetrain() {
    leftDriveTalon = new TalonSRX(MotorIDs.leftDriveTalonID);
    leftDriveVictor = new VictorSPX(MotorIDs.leftDriveVictorID);
    rightDriveTalon = new TalonSRX(MotorIDs.rightDriveTalonID);
    rightDriveVictor = new VictorSPX(MotorIDs.rightDriveVictorID);
    rightDriveVictor.follow(rightDriveTalon);
    leftDriveVictor.follow(leftDriveTalon);
    leftDriveTalon.setInverted(true);
    leftDriveVictor.setInverted(true);
  }

  public Command driveCommand(DoubleSupplier forward, DoubleSupplier rotation) {
    return run(
        () -> {
          double left = 0;
          double right = 0;
          left = forward.getAsDouble();
          right = forward.getAsDouble();
          left -= rotation.getAsDouble();
          right += rotation.getAsDouble();
          left /= 2;
          right /= 2;
          leftDriveTalon.set(ControlMode.PercentOutput, left);
          rightDriveTalon.set(ControlMode.PercentOutput, right);
          SmartDashboard.putNumber("Left Drive Speed", left);
          SmartDashboard.putNumber("Right Drive Speed", right);
        });
  }
  // Auton command to drive forward for a certain amount of time
  public Command goForwardCommand(double speed, double seconds) {
    return Commands.deadline(
        Commands.waitSeconds(seconds),
        runEnd(
            () -> {
              leftDriveTalon.set(ControlMode.PercentOutput, speed);
              rightDriveTalon.set(ControlMode.PercentOutput, speed);
            },
            () -> {
              leftDriveTalon.set(ControlMode.PercentOutput, 0);
              rightDriveTalon.set(ControlMode.PercentOutput, 0);
            }));
  }

  public Command turnCommand(double axis, double seconds) {
    return Commands.deadline(
        Commands.waitSeconds(seconds),
        runEnd(
            () -> {
              leftDriveTalon.set(ControlMode.PercentOutput, axis);
              rightDriveTalon.set(ControlMode.PercentOutput, -axis);
            },
            () -> {
              leftDriveTalon.set(ControlMode.PercentOutput, 0);
              rightDriveTalon.set(ControlMode.PercentOutput, 0);
            }));
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }
}
