package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {
  // Add PIDController calculation to setArmMotor parameter
  private final TalonSRX m_armMotor;
  private final DutyCycleEncoder m_armEncoder;
  public boolean isStowed;
  private final ArmFeedforward feedforwardArm;

  public ArmSubsystem() {
    isStowed = true;
    m_armMotor = new TalonSRX(5);
    m_armEncoder = new DutyCycleEncoder(1);
    m_armEncoder.setDistancePerRotation(Math.PI * 2);
    feedforwardArm =
        new ArmFeedforward(1.0, ArmConstants.kArmG, ArmConstants.kArmV, ArmConstants.kArmA);
  }

  public Command changeArmState() {
    if (isStowed == true) {
      isStowed = false;
      return new PIDCommand(armPIDController
          () -> {
            setArmMotors(feedforwardArm.calculate(m_armEncoder.getAbsolutePosition(), 0, 0)/RobotController.getBatteryVoltage());
          });
    } else {
      isStowed = true;
      return run(
          () -> {
            setArmMotors(feedforwardArm.calculate(m_armEncoder.getAbsolutePosition(), 0, 0)/RobotController.getBatteryVoltage());
          });
    }
  }
  private double getArmAngle () {
    return m_armEncoder.getAbsolutePosition();
  }
  private void setArmMotors(double output) {
    m_armMotor.set(ControlMode.PercentOutput, output);
  }
}
