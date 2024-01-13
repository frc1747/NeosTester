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
  public static class ControllerConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }

  public static class IntakeConstants {

  }

  public static class ShooterConstants {

  }

  public static class DrivetrainConstants {
    public static final int FRONTLEFT_DRIVE_MOTOR = 0;
    public static final int FRONTLEFT_ANGLE_MOTOR = 1;
    public static final int FRONTLEFT_CANCODER = 2;
    public static final int FRONTRIGHT_DRIVE_MOTOR = 3;
    public static final int FRONTRIGHT_ANGLE_MOTOR = 4;
    public static final int FRONTRIGHT_CANCODER = 5;
    public static final int REARLEFT_DRIVE_MOTOR = 6;
    public static final int REARLEFT_ANGLE_MOTOR = 7;
    public static final int REARLEFT_CANCODER = 8;
    public static final int REARRIGHT_DRIVE_MOTOR = 9;
    public static final int REARRIGHT_ANGLE_MOTOR = 10;
    public static final int REARRIGHT_CANCODER = 11;


    // TODO: Tune these later
    public static final double MAX_SPEED = 4.1;  // Max speed in m/s
    public static final double MAX_ACCEL = 4.1;  // Max acceleration in m/s

    // TODO: Tune these later
    public static final double DRIVE_KS = 0.0; 
    public static final double DRIVE_KV = 0.0;
    public static final double DRIVE_KA = 0.0;
  }

  public static class VisionConstants {

  }
}
