// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class Intake extends SubsystemBase {
  private final TalonSRX leftIntakeArmMotor;
  private final TalonSRX rightIntakeArmMotor;

  public Intake() {
    leftIntakeArmMotor = new TalonSRX(MotorIDs.leftIntakeArmMotor);
    rightIntakeArmMotor = new TalonSRX(MotorIDs.rightIntakeArmMotor);
    rightIntakeArmMotor.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intakeBeaker() {
    return startEnd(
        () -> {
          leftIntakeArmMotor.set(ControlMode.PercentOutput, 1);
          rightIntakeArmMotor.set(ControlMode.PercentOutput, 1);
        },
        () -> {
          leftIntakeArmMotor.set(ControlMode.PercentOutput, 0);
          rightIntakeArmMotor.set(ControlMode.PercentOutput, 0);
        });

    // runEnd(() -> {}, ()->{}).andThen(new WaitCommand);

  }

  public Command outtakeBeaker() {
    return Commands.deadline(
        Commands.waitSeconds(1),
        runEnd(
            () -> {
              leftIntakeArmMotor.set(ControlMode.PercentOutput, -1);
              rightIntakeArmMotor.set(ControlMode.PercentOutput, -1);
            },
            () -> {
              leftIntakeArmMotor.set(ControlMode.PercentOutput, 0);
              rightIntakeArmMotor.set(ControlMode.PercentOutput, 0);
            }));

    // runEnd(() -> {}, ()->{}).andThen(new WaitCommand);

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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
