// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class SwerveDriveConstants {
    public static final double robotWidth = 0.762; // Distance between left and right wheels in meters
    public static final double robotLength = 0.762;  // Distance between front and back wheels in meters
    public static final double kMaxSpeedMetersPerSecond = 3.0; // Maximum speed of the robot
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // Maximum angular speed



    public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(robotWidth / 2, robotLength / 2);
    public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(robotWidth / 2, -robotLength / 2);
    public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-robotWidth / 2, robotLength / 2);
    public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-robotWidth / 2, -robotLength / 2);

    public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_LOCATION, FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);


    public static final int TICKS_PER_REVOLUTION = 4096;

    public static final double MAXPERCENTOUTPUT = 0.15; 
    public static final double DRIVER_GEAR_RATIO = 6.86;

    public static final String CANbus = "rio";

    public static final class AngleMotors{
      public static final int FRONT_LEFT_ID = 0; 
      public static final int FRONT_RIGHT_ID = 15; 
      public static final int BACK_LEFT_ID = 2; 
      public static final int BACK_RIGHT_ID = 13;

      public static final double kF = 0.0;
      public static final double kP = 0.3;
      public static final double kI = 0.0;
      public static final double kD = 0.02;
    }

    public static final class DriveMotors{
      public static final int FRONT_LEFT_ID = 1; 
      public static final int FRONT_RIGHT_ID = 14; 
      public static final int BACK_LEFT_ID = 3; 
      public static final int BACK_RIGHT_ID = 12;

      public static final double kF = 0.0;
      public static final double kP = 0.3;
      public static final double kI = 0.0;
      public static final double kD = 0.02;
    }
  }

  public static final class AprilTagConstants {
    public static final double TAG_X = 12.227;
    public static final double TAG_Y = 4.026;
  }

  public static final class PigeonConstants {
    public static final int PIGEON_ID = 10;
  }

  public static final class OIConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final double CONTROLLER_DEADBAND = 0.8;
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
