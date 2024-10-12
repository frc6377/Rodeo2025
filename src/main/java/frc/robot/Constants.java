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

  public static class DrivetrainConstants {
    public static final double maxDrivePercent = 0.525;
  }

  public static class ArmConstants {
    public static final double scoreAngle = Math.toRadians(45.0);
    public static final double stowedAngle = Math.toRadians(130.0);
    public static final double kArmP = 1.0;
    public static final double kArmI = 0.0;
    public static final double kArmD = 0.0;
    public static final double kArmG = 0.3;
    public static final double kArmV = 4.06;
    public static final double kArmA = 0.01;
    public static final double armPercent = .225;
  }

  public static class intakeConstants {
    public static final double intakePercent = .225;
  }
}
