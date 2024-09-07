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
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class PivotConstants {
    public static final int PivotMotorMin = 0;
    public static final int PivotMotorMax = 0;
    public static final double kP = 0.1; // Proportional gain
    public static final double kI = 0.0; // Integral gain
    public static final double kD = 0.0; // Derivative gain
    public static final double kF = 1; // Feedforward gain
    public static final double initalAngle = 20; // Initial angle of the arm
  }

  public static class MotorIDs {
    public static final int leftDriveVictorID = 10;
    public static final int leftDriveTalonID = 9;
    public static final int rightDriveVictorID = 19;
    public static final int rightDriveTalonID = 4;
    public static final int pivotMotorTalonID = 0; // PLACEHOLDER FIXME!!
  }
}
