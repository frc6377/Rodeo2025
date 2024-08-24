package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {

    public boolean isStowed = true;
    private final TalonSRX m_arm = new TalonSRX(5);
    private final Encoder m_armEncoder = new Encoder(0,1);
    //private final PIDController armAnglePidController;

    public Command changeArmState() {
        if(isStowed == true) {
            isStowed = false;
            return run(
                () -> {

                }
            );
        } else {
            isStowed = true;
            return run(
                () -> {

                }
            );
        }
    }
}
