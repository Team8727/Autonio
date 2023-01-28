// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;
import static java.lang.Math.PI;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public enum GamePiece{
    CONE,
    KUBE,
    NONE
  }
  public static final class Controls{
    public static final int primaryController = 0;
  }
  public static final class CanId{
    public static final int leftTalonLead = 1;
    public static final int leftTalonFollow = 2;
    public static final int rightTalonLead = 3;
    public static final int rightTalonFollow = 4;
  }
  public static final class Drivetrain{
    public static final class TrajectoryConstants{
      public static final double kMaxSpeedMetersPerSecond = .75;
      public static final double kMaxAccelerationMetersPerSecondSquared = 1.5;
      // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
      public static final double kRamseteB = 2;
      public static final double kRamseteZeta = 0.7;
    }

    public final class Encoders{
      public static final int leftAPort = 1;
      public static final int leftBPort = 0;
      public static final int rightAPort = 2;
      public static final int rightBPort = 3;

      public static final int PPR = 248;
    }
    public static final class Dimensions{
      public static final double wheelCircumferenceMeters = Units.inchesToMeters(6*Math.PI);
      public static final double trackWidthMeters = Units.inchesToMeters(30);
      public static final boolean kInvertDrive = true;
      //TODO Measure this accurately
      public static final Transform3d aprilTagCameraPositionTransform = new Transform3d( //Cam mounted facing forward, half a meter behind center, half a meter up from center.
        new Translation3d(-0.2794, 0.0, 0.89),
        new Rotation3d(0,-0.0872665,0));
    }

    public static final class Feedforward{
      //Feedforwards from sysid
      public static final class Left{
        public static final double kS = 0.3501;
        public static final double kV = 2.748;
        public static final double kA = 2.3982;
      }
      public static final class Right{
        public static final double kS = 0.49911;
        public static final double kV = 2.499;
        public static final double kA = 1.754;
      }
    }

    public static final class PIDs{
      public static final class Left{
        public static final double kP = 0.1967;
        public static final double kI = 0;
        public static final double kD = 0;
      }
      public static final class Right{
        public static final double kP = 0.22871;
        public static final double kI = 0;
        public static final double kD = 0;
      }
    }

    public static final class Rate{
      //Speeds in m/s rotations in rad/s
      public static final double maxSpeed = 5.45;
      public static final double driverSpeed = 4;
      public static final double driverAngularSpeed = 3;
      public static final double driverAccel = 5;
      public static final double driverDeccel = 10;
    }
  }

  public static final class kSensors{
    public static final int encoderAbsPort = 6;
    public static final int encoderAPort = 4;
    public static final int encoderBPort = 5;
    public static final double distancePerPulse = (2*PI)/2048;
    public static final double distancePerRotation = (2*PI);

    public static final int ledPort = 0;
    public static final int ledLength = 300;
    public static final int proximityThreshold = 100;
  }
}
