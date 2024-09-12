// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
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

  public void runintake(Boolean forward){
    if(forward){
     intakemotorright.set(ControlMode.PercentOutput, 0.7);
     intakemotorleft.set(ControlMode.PercentOutput,0.7);
    }
    else{
      intakemotorright.set(ControlMode.PercentOutput, -0.7);
     intakemotorleft.set(ControlMode.PercentOutput,-0.7);
    }
  public void stopintake (){
    intakemotorright.set(ControlMode.PercentOutput, 0.0);
    intakemotorleft.set(ControlMode.PercentOutput,0.0);
  }


  }
    


  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command intakecommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return startEnd(runintake(true),);
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
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
