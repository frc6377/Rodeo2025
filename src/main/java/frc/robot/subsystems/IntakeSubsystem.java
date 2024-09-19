// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDs;

public class IntakeSubsystem extends SubsystemBase {
  private final VictorSPX intakemotorright;
  private final VictorSPX intakemotorleft;

  /** Creates a new ExampleSubsystem. */
  public IntakeSubsystem() {

    intakemotorright = new VictorSPX(MotorIDs.intakemotorright);
    intakemotorleft = new VictorSPX(MotorIDs.intakemotorleft);
    intakemotorright.setInverted(true);
  }

  public void runintake(Boolean forward) {
    if (forward) {
      intakemotorright.set(ControlMode.PercentOutput, ArmConstants.intakePercent);
      intakemotorleft.set(ControlMode.PercentOutput, ArmConstants.intakePercent);
    } else {
      intakemotorright.set(ControlMode.PercentOutput, -ArmConstants.intakePercent);
      intakemotorleft.set(ControlMode.PercentOutput, -ArmConstants.intakePercent);
    }
  }

  public void stopintake() {
    intakemotorright.set(ControlMode.PercentOutput, 0.0);
    intakemotorleft.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intakeCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.

    return new StartEndCommand(() -> runintake(true), () -> stopintake());
  }

  public Command intakeOutakeCommand() {

    return new StartEndCommand(() -> runintake(false), () -> stopintake());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
