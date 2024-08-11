// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;

public class TestSubsystem extends SubsystemBase {
  private final TalonSRX testMotor1;
  private final TalonSRX testMotor2;
  private final TalonSRX testMotor3;
  private final TalonSRX testMotor4;

  /** Creates a new TestSubsystem. */
  public TestSubsystem() {
    testMotor1 = new TalonSRX(MotorIDs.testMotor1);
    testMotor2 = new TalonSRX(MotorIDs.testMotor2);
    testMotor3 = new TalonSRX(MotorIDs.testMotor3);
    testMotor4 = new TalonSRX(MotorIDs.testMotor4);
  }

  // Function
  public void runMotor1(double percentInput) {
    testMotor1.set(ControlMode.PercentOutput, percentInput);
  }

  public void stopMotor1() {
    testMotor1.set(ControlMode.PercentOutput, 0);
  }

  public void runMotor2(double percentInput) {
    testMotor2.set(ControlMode.PercentOutput, percentInput);
  }

  public void stopMotor2() {
    testMotor1.set(ControlMode.PercentOutput, 0);
  }

  public void runMotor3(double percentInput) {
    testMotor3.set(ControlMode.PercentOutput, percentInput);
  }

  public void stopMotor3() {
    testMotor1.set(ControlMode.PercentOutput, 0);
  }

  public void runMotor4(double percentInput) {
    testMotor4.set(ControlMode.PercentOutput, percentInput);
  }

  public void stopMotor4() {
    testMotor1.set(ControlMode.PercentOutput, 0);
  }

  // Command Factories
  public Command runMotorCommand1() {
    return Commands.startEnd(
        () -> {
          runMotor1(.5);
        },
        () -> {
          stopMotor1();
        },
        this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
