// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveTrainSubsystem extends SubsystemBase {
  private final TalonSRX leftDriveMotor1 = new TalonSRX(0);
  private final TalonSRX leftDriveMotor2 = new TalonSRX(1);
  private final TalonSRX rightDriveMotor1 = new TalonSRX(2);
  private final TalonSRX rightDriveMotor2 = new TalonSRX(3);

  /** Creates a new ExampleSubsystem. */
  public DriveTrainSubsystem() {
    leftDriveMotor2.follow(leftDriveMotor1);
    rightDriveMotor2.follow(rightDriveMotor1);

    rightDriveMotor1.setInverted(true);
    rightDriveMotor2.setInverted(true);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command driveCommand(DoubleSupplier leftY, DoubleSupplier rightX) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          // Max input 0.5, Add axis for turing and driving at the same time

          leftDriveMotor1.set(ControlMode.PercentOutput, leftY.getAsDouble());
          rightDriveMotor1.set(ControlMode.PercentOutput, leftY.getAsDouble());

          leftDriveMotor1.set(ControlMode.PercentOutput, rightX.getAsDouble());
          rightDriveMotor1.set(ControlMode.PercentOutput, rightX.getAsDouble());
        });
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
