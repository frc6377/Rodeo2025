package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class ArmSubsystem extends SubsystemBase {

    public boolean isStowed = true;
    private final TalonSRX m_armMotor = new TalonSRX(5);
    private final Encoder m_armEncoder = new Encoder(0,1);
    private final ArmFeedforward feedforwardUp = new ArmFeedforward(1.0, ArmConstants.kArmG, ArmConstants.kArmV, ArmConstants.kArmA);

    public Command changeArmState() {
        if(isStowed == true) {
            isStowed = false;
            return run(
                () -> {
                    feedforwardUp.calculate(1, 2, 3);
                }
            );
        } else {
            isStowed = true;
            return run(
                () -> {
                    feedforwardUp.calculate(-1, 2, 3);
                }
            );
        }
    }
}
