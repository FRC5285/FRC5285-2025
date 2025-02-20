// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

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

    public static class VisionConstants {
        // ---------------------------------------------------------------------
        // Fill the following with camera names and offsets (from center of bot)
        // ---------------------------------------------------------------------
        // Update num cameras too (important!)
        public static final int numCameras = 0;

        public static final String[] cameraNames = {

        };
        public static final Transform3d[] cameraOffsets = {

        };

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
    }

    public static class AutoConstants {
        // Used for constraints of pathfinding
        public static final double maxVelocityMPS = 5.0; // Max robot speed, m/s

        public static final double maxAccelMPS2 = 4.0; // Max robot accelerations m/s/s

        public static final double maxSpinRadPS = 3.0 * (2.0 * Math.PI); // Max robot angular velocity, radians/s

        public static final double maxSpinAccelRadPS2 = 2.0 * (2.0 * Math.PI); // Max robot angular acceleration, radians/s/s
    }

    public static class RobotConstantsMeters {
        // METERS!!!
        // Half of the robot's width, in meters
        public static final double halfWidth = 0.433;
        // Note: Coral intake is to the LEFT of the robot's center (from the robot's perspective)
        public static final double coralArmOffset = 0.154;

        // Gap between robot and places to score
        public static final double processorSafeDist = 0.1;

        public static final double cageSafeDist = 0.1;

        public static final double reefSafeDist = 0.05;

        public static final double reefAlgaeSafeDist = 0.00; // Gets added to reefSafeDist

        public static final double coralStationSafeDist = 0.05;

        // Gets added onto coralArmOffset
        public static final double coralStationCorrection = 0.0;
        
        public static final double reefBranchCorrection = 0.0;
    }

    public static class FieldConstants {
        // Coordinates are in meters, and for the Pose2d class
        // All distances are in meters

        // Reef measurements
        public static final double blueReefCenterX = 4.5;
        public static final double blueReefCenterY = 4.0;

        public static final double redReefCenterX = 13.06;
        public static final double redReefCenterY = 4.0;

        public static final double reefWallDistance = 0.832;
        public static final double reefCoralOffset = 0.164;

        // Coral Station
        public static final double coralStationBlueX = 0.85;
        public static final double coralStationRedX = 16.7;
        
        public static final double coralStationTopY = 7.38;
        public static final double coralStationBottomY = 0.64;

        public static final double coralStationOffset = 0.406;

        // Algae Processor
        public static final double blueProcessorX = 6.0;
        public static final double blueProcessorY = 0.0;

        public static final double redProcessorX = 11.55;
        public static final double redProcessorY = 8.0;
        
        // Barge and Cages
        public static final double bargeCenterX = 8.775;
        public static final double bargeCenterY = 4.0;

        public static final double firstCageOffsetY = 1.059;
        public static final double cageOffsetY = 1.091;
        public static final double cageOffsetX = 0.093; // half of width of cage
    }

    public static class FlywheelConstants{
        public static final int flywheelMotorID = 6;
        public static final int intakeSensorID = 5;
        public static final double intakeSpeed = -0.1;
        public static final double shootSpeed = 0.1;
        public static final double shootDuration = 0.5;
    }

    public static class WristConstants{
        public static final int wristMotorID = 4;
        public static final int wristEncoderID = 9;
    }
}
