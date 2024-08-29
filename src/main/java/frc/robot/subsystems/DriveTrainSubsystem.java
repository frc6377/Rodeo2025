// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MotorIDs;
import java.util.function.DoubleSupplier;

public class DriveTrainSubsystem extends SubsystemBase {
  private final VictorSPX leftDriveMotor1;
  private final VictorSPX leftDriveMotor2;
  private final VictorSPX rightDriveMotor1;
  private final VictorSPX rightDriveMotor2;

  private final Pigeon2 drivePigeon2;

  /** Creates a new ExampleSubsystem. */
  public DriveTrainSubsystem() {
    leftDriveMotor1 = new VictorSPX(MotorIDs.leftDriveMotor1);

    leftDriveMotor2 = new VictorSPX(MotorIDs.leftDriveMotor2);
    leftDriveMotor2.follow(leftDriveMotor1);

    rightDriveMotor1 = new VictorSPX(MotorIDs.rightDriveMotor1);
    rightDriveMotor1.setInverted(true);

    rightDriveMotor2 = new VictorSPX(MotorIDs.rightDriveMotor2);
    rightDriveMotor2.follow(rightDriveMotor1);
    rightDriveMotor2.setInverted(true);

    drivePigeon2 = new Pigeon2(MotorIDs.Pigeon2ID);
    drivePigeon2.setYaw(0);
  }

  public Trigger isGyroInRange(double target) {
    return new Trigger(
        () ->
            (target - DriveTrainConstants.angleTollerance < drivePigeon2.getYaw().getValueAsDouble()));
  }

  public void setLeftPercent(double percent) {
    leftDriveMotor1.set(ControlMode.PercentOutput, percent);
  }

  public void setRightPercent(double percent) {
    rightDriveMotor1.set(ControlMode.PercentOutput, percent);
  }

  public Command driveCommand(DoubleSupplier leftY, DoubleSupplier rightX) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          double leftPercent =
              (leftY.getAsDouble() * DriveTrainConstants.maxDrivePercent)
                  + (rightX.getAsDouble() * DriveTrainConstants.maxTurnPercent);
          double rightPercent =
              (leftY.getAsDouble() * DriveTrainConstants.maxDrivePercent)
                  + (-rightX.getAsDouble() * DriveTrainConstants.maxDrivePercent);

          leftDriveMotor1.set(ControlMode.PercentOutput, leftPercent);
          rightDriveMotor1.set(ControlMode.PercentOutput, rightPercent);
        });
  }

  // Auto Functions
  public Command setForwardCommand(double sec, double percent) {
    return Commands.deadline(
        Commands.waitSeconds(sec),
        run(
            () -> {
              setLeftPercent(percent);
              setRightPercent(percent);
            }));
  }

  public Command turnLeftCommand(double deg, double percent) {
    return run(() -> {
          setLeftPercent(-percent);
          setRightPercent(percent);
        })
        .onlyWhile(isGyroInRange(deg).negate());
  }

  public Command turnRightCommand(double deg, double percent) {
    return run(() -> {
          setLeftPercent(percent);
          setRightPercent(-percent);
        })
        .onlyWhile(isGyroInRange(deg).negate());
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Drive Left Motor 1", leftDriveMotor1.getMotorOutputPercent());
    SmartDashboard.putNumber("Drive Left Motor 2", leftDriveMotor2.getMotorOutputPercent());
    SmartDashboard.putNumber("Drive Right Motor 1", rightDriveMotor1.getMotorOutputPercent());
    SmartDashboard.putNumber("Drive Right Motor 2", rightDriveMotor2.getMotorOutputPercent());

    SmartDashboard.putNumber("Pigeon Yaw", drivePigeon2.getYaw().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
