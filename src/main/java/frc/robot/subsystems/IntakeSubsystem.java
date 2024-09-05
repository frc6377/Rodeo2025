// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX leftArmMotor;
  private final TalonSRX rightArmMotor;

  public IntakeSubsystem() {
    leftArmMotor = new TalonSRX(MotorIDs.leftArmMotor);

    rightArmMotor = new TalonSRX(MotorIDs.rightArmMotor);
    rightArmMotor.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intake() {

    return runOnce(
            () -> {
              leftArmMotor.set(ControlMode.PercentOutput, 1);
              rightArmMotor.set(ControlMode.PercentOutput, 1);
            })
        .andThen(new WaitCommand(1))
        .andThen(
            runOnce(
                () -> {
                  leftArmMotor.set(ControlMode.PercentOutput, 0);
                  rightArmMotor.set(ControlMode.PercentOutput, 0);
                }));

    // runEnd(() -> {}, ()->{}).andThen(new WaitCommand);

  }

  public Command outtake() {

    return runOnce(
            () -> {
              leftArmMotor.set(ControlMode.PercentOutput, -1);
              rightArmMotor.set(ControlMode.PercentOutput, -1);
            })
        .andThen(new WaitCommand(1))
        .andThen(
            runOnce(
                () -> {
                  leftArmMotor.set(ControlMode.PercentOutput, 0);
                  rightArmMotor.set(ControlMode.PercentOutput, 0);
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
