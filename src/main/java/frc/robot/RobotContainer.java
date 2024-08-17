// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.DriveTrainSubsystem;
import frc.robot.subsystems.EffectorSubsystem;
import frc.robot.subsystems.TestSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveTrainSubsystem m_DriveTrainSubsystem = new DriveTrainSubsystem();
  private final EffectorSubsystem m_EffectorSubsystem = new EffectorSubsystem();
  private final TestSubsystem m_TestSubsystem = new TestSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  private final Joystick m_streamDeck = new Joystick(OperatorConstants.kStreamDeckPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_DriveTrainSubsystem.setDefaultCommand(
        m_DriveTrainSubsystem.driveCommand(
            m_driverController::getLeftY, m_driverController::getRightX));

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_DriveTrainSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_DriveTrainSubsystem));

    // Driver Controls
    m_driverController.a().whileTrue(m_TestSubsystem.runMotorCommand1());

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
    return Autos.exampleAuto(m_DriveTrainSubsystem);
  }
}
