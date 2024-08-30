// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDs;

public class ArmSubsystem extends SubsystemBase {
  private final TalonSRX armMotor1;
  private final TalonSRX armMotor2;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor1 = new TalonSRX(MotorIDs.armMotor1);
    armMotor2 = new TalonSRX(MotorIDs.armMotor2);
    armMotor2.follow(armMotor1);
  }

  public void setArmPose(double point) {
    // Need to figure out what encoders will be used before coding
  }

  // Commands
  public Command scoreLowCommand() {
    return run(
        () -> {
          setArmPose(ArmConstants.lowScorePose);
        });
  }

  public Command scoreHighCommand() {
    return run(
        () -> {
          setArmPose(ArmConstants.highScorePose);
        });
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
