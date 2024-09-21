// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.HowdyPID;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class MotorIDs {
    public static final int leftDriveMotor1 = 2;
    public static final int leftDriveMotor2 = 1;
    public static final int rightDriveMotor1 = 4;
    public static final int rightDriveMotor2 = 3;

    public static final int Pigeon2ID = 5;

    public static final int armMotor1 = 7;
    public static final int armMotor2 = 10;

    public static final int effectorMotor1 = 11;
    // not CAN bus
    public static final int armEncoder = 9;
  }

  public static class DriveTrainConstants {
    // Sum of these value should be <= 1 (can be over, would not cause problems)
    public static final HowdyPID drivePID = new HowdyPID(0.0075, 0, 0);

    public static final double minTurnSpeed = 0.1;

    public static final double maxDrivePercent = 0.5;
    public static final double maxTurnPercent = 0.5;

    public static final double angleTolerance = 2;

    public static final double minPower = 0.3;
    public static final double debounce = 1;
  }

  public static class ArmConstants {
    public static final HowdyPID armPID = new HowdyPID(7.5, 0, 1);

    public static final double kS = 0;
    public static final double kG = 0.14; // 0.55
    public static final double kV = 2.15;
    public static final double kA = 0.01; // 0.05

    public static final double lowScorePose = 2.863492;
    public static final double highScorePose = Units.degreesToRadians(25);

    public static final double offset = 2.789822;

    // Simulation constants
    public static final DCMotor kArmMotor = DCMotor.getCIM(2);
    public static final double kArmGearing = 45;
    // CAD MOI in in^2 lb
    // xx 109.765 (Using)
    // yy 92.943
    // zz 47
    public static final double kArmMOI = 4.625509184446; // Units = m^2 kg
    public static final double kArmMinAngle = Units.degreesToRadians(-115);
    public static final double kArmMaxAngle = Units.degreesToRadians(180);

    public static final double kArmBaseLength = Units.inchesToMeters(25.88);
    public static final double kArmBaseAngle = 63.6;
    public static final double kArmScoringLength = Units.inchesToMeters(25.88);
    public static final double kArmScoringAngle = Units.degreesToRadians(-45);
  }

  public static class EffectorConstants {
    public static final double intakePercent = 0.25;
    public static final double outtakePercent = -0.25;

    public static final boolean isBackUp = false;
    public static final HowdyPID wristPID = new HowdyPID(0.1, 0.0, 0.0);
    public static final double lowScore = Units.degreesToRadians(25);
    public static final double highScore = Units.degreesToRadians(45);
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kStreamDeckPort = 2;
  }
}
