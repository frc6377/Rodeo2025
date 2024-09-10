// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotGearing;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotMotor;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim.KitbotWheelSize;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Robot;
import java.util.function.DoubleSupplier;

public class DriveTrainSubsystem extends SubsystemBase {
  private final VictorSPX leftDriveMotor1;
  private final VictorSPX leftDriveMotor2;
  private final VictorSPX rightDriveMotor1;
  private final VictorSPX rightDriveMotor2;

  private final Pigeon2 drivePigeon2;

  private final PIDController drivePIDController;

  private Double targetAngle;

  // Simulation
  private DifferentialDrivetrainSim m_differentialDrivetrainSim;
  private Field2d m_field;

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

    drivePIDController = DriveTrainConstants.drivePID.getPIDController();
    DriveTrainConstants.drivePID.createTunableNumbers("Drive PID", drivePIDController, this);

    targetAngle = drivePigeon2.getYaw().getValueAsDouble();

    if (Robot.isSimulation()) {
      m_field = new Field2d();
      SmartDashboard.putData("Field", m_field);

      m_differentialDrivetrainSim =
          DifferentialDrivetrainSim.createKitbotSim(
              KitbotMotor.kDualCIMPerSide, // 2 CIMs per side.
              KitbotGearing.k10p71, // 10.71:1
              KitbotWheelSize.kSixInch, // 6" diameter wheels.
              null // No measurement noise.
              );
      m_differentialDrivetrainSim.setPose(new Pose2d(0.0, 4.5, new Rotation2d()));
    }
  }

  public Trigger isGyroInRange(double target) {
    return new Trigger(
        () ->
            targetAngle - DriveTrainConstants.angleTolerance < getDriveAngleDeg()
                && getDriveAngleDeg() < targetAngle + DriveTrainConstants.angleTolerance);
  }

  public void setLeftPercent(double percent) {
    leftDriveMotor1.set(ControlMode.PercentOutput, percent);
  }

  public void setRightPercent(double percent) {
    rightDriveMotor1.set(ControlMode.PercentOutput, percent);
  }

  public double getDriveAngleDeg() {
    if (Robot.isSimulation()) {
      return m_differentialDrivetrainSim.getPose().getRotation().getDegrees();
    } else {
      return drivePigeon2.getYaw().getValueAsDouble();
    }
  }

  public Command driveCommand(DoubleSupplier forwardAxis, DoubleSupplier turnAxis) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return run(
        () -> {
          double leftPercent =
              (forwardAxis.getAsDouble() * DriveTrainConstants.maxDrivePercent)
                  + (turnAxis.getAsDouble() * DriveTrainConstants.maxTurnPercent);
          double rightPercent =
              (forwardAxis.getAsDouble() * DriveTrainConstants.maxDrivePercent)
                  + (-turnAxis.getAsDouble() * DriveTrainConstants.maxDrivePercent);

          leftDriveMotor1.set(ControlMode.PercentOutput, leftPercent);
          rightDriveMotor1.set(ControlMode.PercentOutput, rightPercent);
        });
  }

  // Auto Functions
  public Command setForwardCommand(double sec, double percent) {
    return Commands.deadline(
        Commands.waitSeconds(sec),
        runEnd(
            () -> {
              setLeftPercent(percent);
              setRightPercent(percent);
            },
            () -> {
              setLeftPercent(0);
              setRightPercent(0);
            }));
  }

  public Command turnCommand(double deg) {
    targetAngle = getDriveAngleDeg() - deg;

    return Commands.sequence(
            runOnce(
                () -> {
                  targetAngle = getDriveAngleDeg() - deg;
                }),
            new PIDCommand(
                drivePIDController,
                () -> getDriveAngleDeg(),
                () -> {
                  return targetAngle;
                },
                (output) -> {
                  SmartDashboard.putNumber("PID Output", output);
                  if (Math.abs(output) < DriveTrainConstants.minPower && Math.abs(output) > 0.025) {
                    output = Math.copySign(DriveTrainConstants.minPower, output);
                    setLeftPercent(-output);
                    setRightPercent(output);
                  } else {
                    setLeftPercent(-output);
                    setRightPercent(output);
                  }
                },
                this))
        .until(isGyroInRange(targetAngle).debounce(DriveTrainConstants.debounce))
        .withName("Turn Command");
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

    SmartDashboard.putNumber("Pigeon Yaw", getDriveAngleDeg());
    SmartDashboard.putNumber("Target Angle Auto", targetAngle);
    SmartDashboard.putString(
        "DriveCommand", getCurrentCommand() != null ? getCurrentCommand().getName() : "No Command");
  }

  @Override
  public void simulationPeriodic() {
    m_differentialDrivetrainSim.setInputs(
        leftDriveMotor1.getMotorOutputPercent() * RobotController.getBatteryVoltage(),
        rightDriveMotor1.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    m_differentialDrivetrainSim.update(0.02);

    m_field.setRobotPose(m_differentialDrivetrainSim.getPose());
  }
}
