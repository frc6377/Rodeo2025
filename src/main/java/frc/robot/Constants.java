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
    public static final double kS = 2; // Static gain
    public static final double kG = 0.5; // Gravity gain
    public static final double kV = 2.15; // Velocity gain
    public static final double kA = 0.04; // Acceleration gain
    public static final double initalAngle = 0; // Initial angle of the arm
    public static final double scoreHighAngle = -120; // Angle to score high
    public static final double scoreLowAngle = -45; // Angle to score low
    public static final double pickUpBeakerAngle = -20; // Angle to intake
  }

  public static class MotorIDs {
    public static final int leftDriveVictorID = 10;
    public static final int leftDriveTalonID = 9;
    public static final int rightDriveVictorID = 19;
    public static final int rightDriveTalonID = 4;
    public static final int pivotMotorMasterID = 7;
    public static final int pivotMotorSlaveID = 5;
    public static final int IntakeMotorID = 2;
  }
}
