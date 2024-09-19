package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
  private final TalonSRX m_frontLeftMotor = new TalonSRX(0);
  private final TalonSRX m_frontRightMotor = new TalonSRX(1);
  private final TalonSRX m_BackLeftMotor = new TalonSRX(2);
  private final TalonSRX m_BackRightMotor = new TalonSRX(3);

  public Drivetrain() {
    m_BackRightMotor.follow(m_frontRightMotor);
    m_BackLeftMotor.follow(m_frontLeftMotor);
    m_frontLeftMotor.setInverted(true);
    m_BackLeftMotor.setInverted(true);
  }

  public Command driveCommand(DoubleSupplier leftY, DoubleSupplier rightY) {
    return run(
        () -> {
          double leftPercent = (leftY.getAsDouble() * DrivetrainConstants.maxDrivePercent);
          double rightPercent = (rightY.getAsDouble() * DrivetrainConstants.maxDrivePercent);

          m_frontLeftMotor.set(ControlMode.PercentOutput, leftPercent);
          m_frontRightMotor.set(ControlMode.PercentOutput, rightPercent);
        });
  }

  public Command Forward(double sec, double percent) {
    return Commands.deadline(
        Commands.waitSeconds(sec),
        runEnd(
            () -> {
              m_frontLeftMotor.set(ControlMode.PercentOutput, percent);
              m_frontRightMotor.set(ControlMode.PercentOutput, percent);
            },
            () -> {
              m_frontLeftMotor.set(ControlMode.PercentOutput, 0);
              m_frontRightMotor.set(ControlMode.PercentOutput, 0);
            }));
  }
}
