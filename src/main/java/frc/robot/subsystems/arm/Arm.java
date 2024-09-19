// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MotorIDs;
import frc.robot.Constants.PivotConstants;
import frc.robot.utilities.DebugEntry;
import java.util.function.DoubleSupplier;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */
  private final TalonSRX masterPivotMotor;

  private final TalonSRX slavePivotMotor;
  private final DutyCycleEncoder pivotEncoder;
  private double targetAngle = PivotConstants.initalAngle;
  private final double minAngle = PivotConstants.PivotMotorMin;
  private final double maxAngle = PivotConstants.PivotMotorMax;
  private PIDController pidController =
      new PIDController(PivotConstants.kP, PivotConstants.kI, PivotConstants.kD);
  private DebugEntry<Double> m_kP;
  private DebugEntry<Double> m_kI;
  private DebugEntry<Double> m_kD;
  public Arm() {
    pivotEncoder = new DutyCycleEncoder(0);
    pivotEncoder.setDistancePerRotation(360.0);
    masterPivotMotor = new TalonSRX(MotorIDs.pivotMotorMasterID);
    masterPivotMotor.configFactoryDefault();
    masterPivotMotor.setNeutralMode(NeutralMode.Brake);
    slavePivotMotor = new TalonSRX(MotorIDs.pivotMotorSlaveID);
    slavePivotMotor.configFactoryDefault();
    slavePivotMotor.setNeutralMode(NeutralMode.Brake);
    slavePivotMotor.setInverted(true);
    slavePivotMotor.follow(masterPivotMotor);

    pivotEncoder.reset();

    m_kP = new DebugEntry<>(PivotConstants.kP, "kP", this);
    m_kI = new DebugEntry<>(PivotConstants.kI, "kI", this);
    m_kD = new DebugEntry<>(PivotConstants.kD, "kD", this);
  }
  /**
   * Gets the current angle of the arm
   *
   * @return the value of the arm encoder in rotations
   */
  public double getCurrentAngle() {
    // Get the current angle of the pivotn in rotations
    return pivotEncoder.get();
  }

  public Command changeTargetAngle(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    goToAngle(0).schedule();
    return run(
        () -> {
          double angle =
              MathUtil.applyDeadband(
                      ((rightTrigger.getAsDouble() - leftTrigger.getAsDouble()) * 10), 0.1)
                  + targetAngle;
          goToAngle(angle);
        });
  }

  public Command goToAngle(double angle) {
    return run(
        () -> {
          targetAngle = Math.max(minAngle, Math.min(maxAngle, angle));
        });
  }

  public Command scoreHighCommand() {
    return run(
        () -> {
          targetAngle = PivotConstants.scoreHighAngle;
        });
  }

  public Command scoreLowCommand() {
    return run(
        () -> {
          targetAngle = PivotConstants.scoreLowAngle;
        });
  }

  public Command pickUpBeakerCommand() {
    return run(
        () -> {
          targetAngle = PivotConstants.pickUpBeakerAngle;
        });
  }

  private void update() {
    double currentAngle = getCurrentAngle();
    double pidOutput = pidController.calculate(currentAngle, targetAngle); // Calculate PID output
    masterPivotMotor.set(ControlMode.PercentOutput, pidOutput);
  }

  public Command runRaw(double speed) {
    return run(
        () -> {
          masterPivotMotor.set(ControlMode.PercentOutput, speed);
        });
  }

  @Override
  public void periodic() {
    update();
    m_kP.log(pidController.getP());
    m_kI.log(pidController.getI());
    m_kD.log(pidController.getD());

    pidController.setP(m_kP.get());
    pidController.setI(m_kI.get());
    pidController.setD(m_kD.get());
    SmartDashboard.putNumber("Current Angle", (getCurrentAngle() * 360) % 360);
    SmartDashboard.putNumber("Target Angle", targetAngle);
    SmartDashboard.putBoolean("Arm at taget angle", getCurrentAngle() == targetAngle);
  }
}
