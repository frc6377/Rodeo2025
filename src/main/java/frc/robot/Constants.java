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

  public static class MotorIDs {
    public static final int leftDriveMotor1 = 9;
    public static final int leftDriveMotor2 = 19;
    public static final int rightDriveMotor1 = 4;
    public static final int rightDriveMotor2 = 10;
    public static final int rightArmMotor = 5; // TODO: change after actual controllers
    public static final int leftArmMotor = 8; // TODO: change after actual controllers
  }

  public static class DriveTrainConstants {
    public static final double maxDrivePercent = 1;
    public static final double maxTurnPercent = 1;
  }
}
