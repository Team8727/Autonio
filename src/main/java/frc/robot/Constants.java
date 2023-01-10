// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Canid{
    public static final int leftTalonLead = 2;
    public static final int leftTalonFollow = 3;
    public static final int rightTalonLead = 0;
    public static final int rightTalonFollow = 1;
  }
  public static final class Drivetrain{
    public final class Encoders{
      public static final int rightAPort = 2;
      public static final int rightBPort = 3;
      public static final int leftBPort = 0;
      public static final int leftAPort = 1;

      public static final int PPR = 248;
    }
    public static final class Dimensions{
      public static final double wheelCircumferenceMeters = Units.inchesToMeters(6*Math.PI);
      public static final double trackWidthMeters = Units.inchesToMeters(30);
      //TODO Measure this accurately
      public static final Transform3d aprilTagCameraPositionTransform = new Transform3d( //Cam mounted facing forward, half a meter behind center, half a meter up from center.
        new Translation3d(-0.5, 0.0, 0.5),
        new Rotation3d(0,0,0));
    }
  }
}
