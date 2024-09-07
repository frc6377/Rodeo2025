// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;  
import java.util.function.DoubleSupplier;
import frc.robot.Constants.PivotConstants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonSRX pivotMotor;
  private double targetAngle = PivotConstants.initalAngle;
  private double kP = PivotConstants.kP; // Proportional gain
  private double kI = PivotConstants.kI; // Integral gain
  private double kD = PivotConstants.kD; // Derivative gain
  private double kF = PivotConstants.kF; // Feedforward gain
  private double dt = 0.05; // Time in seconds between each update
  private double minAngle = PivotConstants.PivotMotorMin;
  private double maxAngle = PivotConstants.PivotMotorMax;

  public Arm() {
      pivotMotor = new TalonSRX(MotorIDs.pivotMotorTalonID);
      pivotMotor.configFactoryDefault();
      pivotMotor.setNeutralMode(NeutralMode.Brake);
      pivotMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 0);
      pivotMotor.config_kP(0, kP);
      pivotMotor.config_kI(0, kI);
      pivotMotor.config_kD(0, kD);
      pivotMotor.config_kF(0, kF);

    }
    public double getCurrentAngle() {
      return pivotMotor.getSelectedSensorPosition(0);
    }
    public Command setTargetAngle(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
      return run(
        ()->{
      double angle = ((rightTrigger.getAsDouble() - leftTrigger.getAsDouble())*2)+getCurrentAngle();
      targetAngle = Math.max(minAngle, Math.min(maxAngle, angle));
        });
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
