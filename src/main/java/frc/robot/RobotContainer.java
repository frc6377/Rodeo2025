// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.subsystems.arm.Arm;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.intake.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final Drivetrain m_DrivetrainSubsystem;
  private final Arm m_ArmSubsystem;
  private final Intake m_IntakeSubsystem;
  private final ShuffleboardTab configTab = Shuffleboard.getTab("Config");
  private final SendableChooser<Command> m_chooser;
  // Replace with CommandPS4Controller or CommandJoystick if needed

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    m_DrivetrainSubsystem = new Drivetrain();
    m_ArmSubsystem = new Arm();
    m_IntakeSubsystem = new Intake();

    m_chooser = new SendableChooser<>();
    Command scoreHigh =
        Autos.scoreHighAutoCommand(m_DrivetrainSubsystem, m_ArmSubsystem, m_IntakeSubsystem);
    Command scoreLow =
        Autos.scoreLowAutoCommand(m_DrivetrainSubsystem, m_ArmSubsystem, m_IntakeSubsystem);
    Command pickUpBeaker =
        Autos.pickUpBeakerAutoCommand(m_DrivetrainSubsystem, m_ArmSubsystem, m_IntakeSubsystem);
    m_chooser.addOption("Score High", scoreHigh);
    m_chooser.addOption("Score Low", scoreLow);
    m_chooser.addOption("Pick Up Beaker", pickUpBeaker);
    m_chooser.setDefaultOption("Score High", scoreHigh);
    configTab.add("Autonomous Selection", m_chooser);
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

    m_driverController.leftBumper().whileTrue(m_IntakeSubsystem.intakeBeaker());
    m_driverController.rightBumper().whileTrue(m_IntakeSubsystem.outtakeBeaker());
    m_driverController.y().whileTrue(m_ArmSubsystem.scoreHighCommand());
    m_driverController.b().whileTrue(m_ArmSubsystem.scoreLowCommand());
    m_driverController.a().whileTrue(m_ArmSubsystem.pickUpBeakerCommand());
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_DrivetrainSubsystem.setDefaultCommand(
        m_DrivetrainSubsystem.driveCommand(
            m_driverController::getLeftY, m_driverController::getRightX));
    m_ArmSubsystem.setDefaultCommand(
        m_ArmSubsystem.changeTargetAngle(
            m_driverController::getLeftTriggerAxis, m_driverController::getRightTriggerAxis));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
