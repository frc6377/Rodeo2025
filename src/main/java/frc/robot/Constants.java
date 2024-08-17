// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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
    public static final int leftDriveMotor1 = 0;
    public static final int leftDriveMotor2 = 1;
    public static final int rightDriveMotor1 = 2;
    public static final int rightDriveMotor2 = 3;

    public static final int effectorMotor1 = 4;
    public static final int effectorMotor2 = 5;

    public static final int testMotor1 = 6;
    public static final int testMotor2 = 7;
    public static final int testMotor3 = 8;
    public static final int testMotor4 = 9;
  }

  public static class DriveTrainConstants {
    // Sum of these value should be <= 1 (can be over, would not cause problems)
    public static final double maxDrivePercent = 0.5;
    public static final double maxTurnPercent = 0.5;
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
