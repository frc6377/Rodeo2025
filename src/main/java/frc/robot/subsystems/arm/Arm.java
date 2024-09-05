// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.PivotRange;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonSRX pivotMotor;
  private double targetAngle = 0;
  private double kP = 0.1; // Proportional gain
  private double kI = 0.0; // Integral gain
  private double kD = 0.0; // Derivative gain
  private double kF = 0.0; // Feedforward gain
  private double dt = 0.05; // Time in seconds between each update
  private double minAngle = PivotRange.PivotMotorMin;
  private double maxAngle = PivotRange.PivotMotorMax;

  public Arm() {
    pivotMotor = new TalonSRX(MotorIDs.pivotMotorTalonID);
    pivotMotor.configFactoryDefault();
        pivotMotor.setNeutralMode(NeutralMode.Brake);
        pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
        pivotMotor.config_kP(0, kP);
        pivotMotor.config_kI(0, kI);
        pivotMotor.config_kD(0, kD);
        pivotMotor.config_kF(0, kF);

        // Configure follower Talon
    }

    public void setTargetAngle(DoubleSupplier angle) {
      targetAngle = Math.max(minAngle, Math.min(maxAngle, angle.getAsDouble()));
    }

    public void update() {
        double currentAngle = pivotMotor.getSelectedSensorPosition(0);
        double error = targetAngle - currentAngle;

        double feedforward = kF * targetAngle; // Calculate feedforward based on target angle
        double pidOutput = kP * error + kI * (error * dt) + kD * (error / dt);

        pivotMotor.set(ControlMode.PercentOutput, feedforward + pidOutput);
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
