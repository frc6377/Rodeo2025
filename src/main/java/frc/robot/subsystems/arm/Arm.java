// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.PivotConstants;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonSRX masterPivotMotor;

  private final TalonSRX slavePivotMotor;
  private final Encoder pivotEncoder;
  private double targetAngle = PivotConstants.initalAngle;
  private double speed = 0;
  private final double minAngle = PivotConstants.PivotMotorMin;
  private final double maxAngle = PivotConstants.PivotMotorMax;
  private PIDController pidController =
      new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
  private ArmFeedforward armFeedforward =
      new ArmFeedforward(
          PivotConstants.kS, PivotConstants.kG, PivotConstants.kV, PivotConstants.kA);

  public Arm() {
    pivotEncoder = new Encoder(0, 1);
    masterPivotMotor = new TalonSRX(MotorIDs.pivotMotorTalonID);
    masterPivotMotor.configFactoryDefault();
    masterPivotMotor.setNeutralMode(NeutralMode.Brake);
    slavePivotMotor = new TalonSRX(MotorIDs.pivotMotorTalonID);
    slavePivotMotor.configFactoryDefault();
    slavePivotMotor.setNeutralMode(NeutralMode.Brake);
    slavePivotMotor.setInverted(true);
    slavePivotMotor.follow(masterPivotMotor);

    pivotEncoder.reset();
    pivotEncoder.setDistancePerPulse(1);
  }

  public double getCurrentAngle() {
    // Placeholder for actual sensor FIXME!!
    return pivotEncoder.getDistance();
  }

  public Command setTargetAngle(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    return run(
        () -> {
          double angle =
              ((rightTrigger.getAsDouble() - leftTrigger.getAsDouble()) * 2) + getCurrentAngle();
          targetAngle = Math.max(minAngle, Math.min(maxAngle, angle));
          update();
        });
  }

  public void update() {
    double currentAngle = getCurrentAngle();
    double feedforward = armFeedforward.calculate(currentAngle, 0); // Calculate feedforward
    double pidOutput = pidController.calculate(currentAngle, targetAngle); // Calculate PID output
    speed = feedforward + pidOutput;
    masterPivotMotor.set(ControlMode.PercentOutput, feedforward/RobotController.getBatteryVoltage() + pidOutput);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
