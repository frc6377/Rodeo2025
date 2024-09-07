// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
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
          left = forward.getAsDouble()*DrivetrainConstants.maxSpeed;
          right = forward.getAsDouble()*DrivetrainConstants.maxSpeed;
          left -= rotation.getAsDouble()*DrivetrainConstants.maxAngularSpeed;
          right += rotation.getAsDouble()*DrivetrainConstants.maxAngularSpeed;
          leftDriveTalon.set(ControlMode.PercentOutput, left);
          rightDriveTalon.set(ControlMode.PercentOutput, right);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("leftDriveTalon Output", leftDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("leftDriveVictor Output", leftDriveVictor.getMotorOutputPercent());
    SmartDashboard.putNumber("rightDriveTalon Output", rightDriveTalon.getMotorOutputPercent());
    SmartDashboard.putNumber("rightDriveVictor Output", rightDriveVictor.getMotorOutputPercent());
  }
}
