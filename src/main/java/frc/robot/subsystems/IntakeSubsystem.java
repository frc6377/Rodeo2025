package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.intakeConstants;
import java.util.function.DoubleSupplier;

public class IntakeSubsystem extends SubsystemBase {
  private final TalonSRX m_intakeMotor;

  public IntakeSubsystem() {
    m_intakeMotor = new TalonSRX(5);
  }

  public Command intakeCommand(DoubleSupplier leftTrigger, DoubleSupplier rightTrigger) {
    return run(
        () -> {
          double intakePercent =
              -((leftTrigger.getAsDouble() - rightTrigger.getAsDouble())
                  * intakeConstants.intakePercent);
          m_intakeMotor.set(ControlMode.PercentOutput, intakePercent);
        });
  }
}
