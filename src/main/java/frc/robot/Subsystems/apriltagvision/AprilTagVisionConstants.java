// Copyright (c) 2024 FRC 6328
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.Subsystems.apriltagvision;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class AprilTagVisionConstants {
  public static final double ambiguityThreshold = 0.4;
  public static final double targetLogTimeSecs = 0.1;
  public static final double fieldBorderMargin = 0.5;
  public static final double zMargin = 0.75;
  public static final double xyStdDevCoefficient = 0.005;
  public static final double thetaStdDevCoefficient = 0.01;

  public static final double[] stdDevFactors =
         new double[] {1.0};

  public static final Pose3d[] cameraPoses =
            new Pose3d[] {
              new Pose3d(
                  Units.inchesToMeters(8.875),
                  Units.inchesToMeters(10.5),
                  Units.inchesToMeters(8.25),
                  new Rotation3d(0.0, Units.degreesToRadians(-28.125), 0.0)
                      .rotateBy(new Rotation3d(0.0, 0.0, Units.degreesToRadians(30.0))))
              };

  public static final String[] instanceNames =
        new String[] {"northstar0"};

  public static final String[] cameraIds =
            new String[] {
              "cam0"
            };
  
  public static final int[] fps= new int[] {50};
};
