package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  // Add PIDController calculation to setArmMotor parameter
  private final TalonSRX m_armMotor;
  // private final DutyCycleEncoder m_armEncoder;
  // public boolean isStowed;
  // private final ArmFeedforward feedforwardArm;
  // private final PIDController armPidController;

  public ArmSubsystem() {
    // isStowed = false;
    m_armMotor = new TalonSRX(6);
    /*m_armEncoder = new DutyCycleEncoder(1);
    m_armEncoder.reset();
    // TODO: add amout for proper offset so FF works.
    m_armEncoder.setDistancePerRotation(Math.PI * 2);
    armPidController =
        new PIDController(ArmConstants.kArmP, ArmConstants.kArmI, ArmConstants.kArmD);
    feedforwardArm =
        new ArmFeedforward(1.0, ArmConstants.kArmG, ArmConstants.kArmV, ArmConstants.kArmA);*/
  }

  public Command armDownCommand() {
    return run(
        () -> {
          double armPercent = ArmConstants.armPercent;

          m_armMotor.set(ControlMode.PercentOutput, armPercent);
        });
  }

  public Command armUpCommand() {
    return run(
        () -> {
          double armPercent = -ArmConstants.armPercent;

          m_armMotor.set(ControlMode.PercentOutput, armPercent);
        });
  }

  public Command armStopCommand() {
    return run(
        () -> {
          m_armMotor.set(ControlMode.PercentOutput, 0);
        });
  }
  /*public Command changeArmState() {
    return Commands.runOnce(() -> isStowed = !isStowed, this)
        .andThen(
            new PIDCommand(
                armPidController,
                this::getArmAngle,
                () -> {
                  if (!isStowed) {
                    SmartDashboard.putNumber("armSetPoint", ArmConstants.scoreAngle);
                    return ArmConstants.scoreAngle;
                  } else {
                    SmartDashboard.putNumber("armSetPoint", ArmConstants.stowedAngle);
                    return ArmConstants.stowedAngle;
                  }
                },
                (a) -> {
                  setArmMotors(
                      a
                          + feedforwardArm.calculate(getArmAngle(), 0, 0)
                              / RobotController.getBatteryVoltage());
                },
                this));
  }

  private double getArmAngle() {
    return m_armEncoder.getDistance();
  }

  private void setArmMotors(double output) {
    m_armMotor.set(ControlMode.PercentOutput, output);
  }*/
}
