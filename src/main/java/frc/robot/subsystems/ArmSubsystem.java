// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.MotorIDs;
import frc.robot.Robot;

public class ArmSubsystem extends SubsystemBase {
  private final TalonSRX armMotor1;
  private final TalonSRX armMotor2;

  private final CANcoder armEncoder;

  private final PIDController armPIDController;
  private final ArmFeedforward armFeedforward;

  private final ShuffleboardTab armTab = Shuffleboard.getTab("Arm");

  // Simulation Things
  private SingleJointedArmSim armSim;
  private Mechanism2d armMechanism2d;
  private MechanismRoot2d root;
  private MechanismLigament2d baseMech;
  private MechanismLigament2d scoringMech;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor1 = new TalonSRX(MotorIDs.armMotor1);
    armMotor2 = new TalonSRX(MotorIDs.armMotor2);
    armMotor2.follow(armMotor1);

    armEncoder = new CANcoder(MotorIDs.armEncoder);
    EncoderSim armEncoderSim = new EncoderSim(new Encoder(0, 1));

    armPIDController = ArmConstants.armPID.getPIDController();
    ArmConstants.armPID.createTunableNumbers("Arm PID", armPIDController, this);

    armFeedforward =
        new ArmFeedforward(ArmConstants.kS, ArmConstants.kG, ArmConstants.kV, ArmConstants.kA);

    if (Robot.isSimulation()) {
      armSim =
          new SingleJointedArmSim(
              ArmConstants.kArmMotor,
              ArmConstants.kArmGearing,
              ArmConstants.kArmMOI,
              ArmConstants.kArmScoringLength,
              ArmConstants.kArmMinAngle,
              ArmConstants.kArmMaxAngle,
              true,
              Units.degreesToRadians(-90));
      armMechanism2d = new Mechanism2d(1, 1);
      root = armMechanism2d.getRoot("Root", 0.25, 0);
      baseMech =
          root.append(
              new MechanismLigament2d(
                  "Base Elv",
                  ArmConstants.kArmBaseLength,
                  ArmConstants.kArmBaseAngle,
                  20,
                  new Color8Bit(Color.kBlue)));
      scoringMech =
          baseMech.append(
              new MechanismLigament2d(
                  "Scoring Elv",
                  ArmConstants.kArmScoringLength,
                  0,
                  10,
                  new Color8Bit(Color.kAqua)));

      armTab.add("Arm Mech", armMechanism2d);
    }
  }

  public void setArmMotors(double output) {
    armMotor1.set(ControlMode.PercentOutput, output);
  }

  public double calcArmFeedforward() {
    double calc =
        armFeedforward.calculate(getArmPoseRads(), armEncoder.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Arm Feedforward", calc);
    return calc;
  }

  public double getArmPoseRads() {
    if (Robot.isSimulation()) {
      return armSim.getAngleRads();
    } else {
      return Units.rotationsToRadians(armEncoder.getPosition().getValueAsDouble());
    }
  }

  public Command setArmVelocity(double velocity) {
    return run(() -> setArmMotors(velocity));
  }

  public Command setArmPosition(double position) {
    SmartDashboard.putNumber("Arm Target", Units.radiansToDegrees(position));
    return new PIDCommand(
        armPIDController,
        () -> getArmPoseRads(),
        position,
        (output) -> setArmMotors(output + calcArmFeedforward()),
        // (output) -> setArmMotors(calcArmFeedforward()),
        this);
  }

  // Commands
  public Command scoreLowCommand() {
    return setArmPosition(ArmConstants.lowScorePose);
  }

  public Command scoreHighCommand() {
    return setArmPosition(ArmConstants.highScorePose);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Arm Motor 1 Output", armMotor1.getMotorOutputPercent());
    SmartDashboard.putNumber("Arm Motor 2 Output", armMotor2.getMotorOutputPercent());

    SmartDashboard.putNumber(
        "Arm Encoder Angle Deg",
        Units.rotationsToDegrees(armEncoder.getPosition().getValueAsDouble()));
    SmartDashboard.putNumber("Arm Encoder Velocity", armEncoder.getVelocity().getValueAsDouble());
  }

  @Override
  public void simulationPeriodic() {
    armSim.setInput(armMotor1.getMotorOutputPercent() * RobotController.getBatteryVoltage());
    armSim.update(Robot.defaultPeriodSecs);

    scoringMech.setAngle(Units.radiansToDegrees(getArmPoseRads()) - ArmConstants.kArmBaseAngle);
    armEncoder.setPosition(Units.radiansToRotations(getArmPoseRads()));

    SmartDashboard.putNumber("Arm Sim Angle", Units.radiansToDegrees(getArmPoseRads()));
  }
}
