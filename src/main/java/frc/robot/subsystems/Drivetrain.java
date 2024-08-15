package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DrivetrainConstants;
import java.util.function.DoubleSupplier;

public class Drivetrain extends SubsystemBase {
  private final TalonSRX m_frontLeftMotor = new TalonSRX(0);
  private final TalonSRX m_frontRightMotor = new TalonSRX(1);
  private final TalonSRX m_BackLeftMotor = new TalonSRX(2);
  private final TalonSRX m_BackRightMotor = new TalonSRX(3);

  public Drivetrain() {
    m_frontRightMotor.setInverted(true);
    m_BackRightMotor.follow(m_frontRightMotor);
    m_BackLeftMotor.follow(m_frontLeftMotor);
  }

  public Command driveCommand(DoubleSupplier leftY, DoubleSupplier rightX) {
    return run(
        () -> {
          double leftPercent =
              (leftY.getAsDouble() * DrivetrainConstants.maxDrivePercent)
                  + (rightX.getAsDouble() * DrivetrainConstants.maxTurningPercent);
          double rightPercent =
              (leftY.getAsDouble() * DrivetrainConstants.maxDrivePercent)
                  + (-rightX.getAsDouble() * DrivetrainConstants.maxDrivePercent);

          m_frontLeftMotor.set(ControlMode.PercentOutput, leftPercent);
          m_frontRightMotor.set(ControlMode.PercentOutput, rightPercent);
        });
  }
}
