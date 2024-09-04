// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;

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

    public static final int armMotor1 = 6;
    public static final int armMotor2 = 7;
    public static final int armEncoder = 8;

    public static final int effectorMotor1 = 9;
    public static final int effectorMotor2 = 10;
  }

  public static class DriveTrainConstants {
    // Sum of these value should be <= 1 (can be over, would not cause problems)
    public static final double kP = 0.015;

    public static final double minTurnSpeed = 0.1;

    public static final double maxDrivePercent = 0.5;
    public static final double maxTurnPercent = 0.5;

    public static final double angleTolerance = 1;
  }

  public static class ArmConstants {
    public static final double kP = 0.25;
    public static final double kI = 0.025;
    public static final double kD = 0.0025;

    public static final double kS = 0;
    public static final double kG = 1.13;
    public static final double kV = 2.15;
    public static final double kA = 0.1;

    public static final double lowScorePose = Units.degreesToRadians(-90);
    public static final double highScorePose = Units.degreesToRadians(25);

    // Simulation constants
    public static final DCMotor kArmMotor = DCMotor.getCIM(2);
    public static final double kArmGearing = 4;
    public static final double kArmMOI = 63.958;
    public static final double kArmMinAngle = Units.degreesToRadians(-115);
    public static final double kArmMaxAngle = Units.degreesToRadians(180);

    public static final double kArmBaseLength = 25.88;
    public static final double kArmBaseAngle = 63.6;
    public static final double kArmScoringLength = 25.698;
    public static final double kArmScoringAngle = Units.degreesToRadians(-45);
  }

  public static class EffectorConstants {
    public static final double intakePercent = 0.25;
    public static final double outakePercent = 0.25;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final int kStreamDeckPort = 2;
  }
}
