// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EffectorSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SendableChooser<Command> autoChooser;
  private final ShuffleboardTab configTab = Shuffleboard.getTab("Config");

  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem m_DriveTrainSubsystem = new DriveTrainSubsystem();
  private final ArmSubsystem m_ArmSubsystem = new ArmSubsystem();
  private final EffectorSubsystem m_EffectorSubsystem = new EffectorSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final Joystick m_streamDeck = new Joystick(OperatorConstants.kStreamDeckPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    if (Robot.isReal()) {
      m_DriveTrainSubsystem.setDefaultCommand(
          m_DriveTrainSubsystem.driveCommand(
              m_driverController::getLeftY, m_driverController::getRightX));
    } else {
      m_DriveTrainSubsystem.setDefaultCommand(
          m_DriveTrainSubsystem.driveCommand(
              m_driverController::getLeftY, m_driverController::getLeftX));
    }

    autoChooser = new SendableChooser<>();

    Command auto1 =
        Autos.OneBeackerAuto(m_DriveTrainSubsystem, m_ArmSubsystem, m_EffectorSubsystem);
    autoChooser.addOption(auto1.getName(), auto1);

    Command auto2 = Autos.AutoTurn90Test(m_DriveTrainSubsystem);
    autoChooser.addOption(auto2.getName(), auto2);

    Command auto3 = Autos.AutoForwardTest(m_DriveTrainSubsystem);
    autoChooser.addOption(auto3.getName(), auto3);

    configTab.add("Auton Selection", autoChooser);

    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Driver Controls
    if (Robot.isSimulation()) {
      m_driverController
          .button(1)
          .whileTrue(m_ArmSubsystem.scoreHighCommand())
          .onFalse(m_ArmSubsystem.setArmVelocity(0));
      m_driverController
          .button(2)
          .whileTrue(m_ArmSubsystem.setArmVelocity(-1))
          .onFalse(m_ArmSubsystem.setArmVelocity(0));
    } else {
      m_driverController
          .leftBumper()
          .whileTrue(m_ArmSubsystem.setArmVelocity(1))
          .onFalse(m_ArmSubsystem.setArmVelocity(0));
      m_driverController
          .rightBumper()
          .whileTrue(m_ArmSubsystem.setArmVelocity(-1))
          .onFalse(m_ArmSubsystem.setArmVelocity(0));
    }

    // Operator Controls
    m_operatorController
        .povUp()
        .whileTrue(m_EffectorSubsystem.intakeCommand())
        .onFalse(m_EffectorSubsystem.stopCommand());
    m_operatorController
        .povDown()
        .whileTrue(m_EffectorSubsystem.outakeCommand())
        .onFalse(m_EffectorSubsystem.stopCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
