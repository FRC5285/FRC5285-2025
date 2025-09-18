// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
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

        public static final int kSecondaryControllerPort = 1;

        public static final double maxSpeedMultiplier = 1.0; // Set this value within the range [0, 1]

        public static final double throttleMaxReduction = 0.9; // Most speed the throttle can take off (range [0, 1])

        public static final double accelLimit = 1.5; // gets to max speed in 1/accelLimit seconds

        public static final double rotLimit = 2.0; // gets to max speed in 1/rotLimit seconds
    }

    public static class VisionConstants {
        // ---------------------------------------------------------------------
        // Fill the following with camera names and offsets (from center of bot)
        // ---------------------------------------------------------------------
        // Update num cameras too (important!)
        public static final int numCameras = 4;

        public static final String[] cameraNames = {
            "Arducam_OV9281_USB1",
            "Arducam_OV9281_USB2",
            "Arducam_OV9281_USB3",
            "Arducam_OV9281_USB4"
        }; //
        public static final Transform3d[] cameraOffsets = {
            new Transform3d(new Translation3d(-0.0592, -0.30164, 0.21672), new Rotation3d(0.0, -0.2 * Math.PI, 0.0)), // On the side beam, facing up 36 deg, tilted 90 deg
            new Transform3d(new Translation3d(-0.26145, -0.2887, 0.19692), new Rotation3d(0.0, (-1.0 / 12.0) * Math.PI, Math.PI)), // Facing backwards
            new Transform3d(new Translation3d(0.28898, -0.11277, 0.18288), new Rotation3d(0.0, (-1.0 / 12.0) * Math.PI, 15.0 / 180.0 * Math.PI)), // Right side, 15 deg to left
            new Transform3d(new Translation3d(0.26245, 0.3026, 0.19322), new Rotation3d(0.0, (-1.0 / 12.0) * Math.PI, 0.0)) // Left side, swerve, facing forward
        };

        public static final Set<Integer> normalShutOffCams = Set.of(); // Indices cameras to shut off (ex: Set.of(0, 1) creates set with 0 and 1).
        public static final Set<Integer> reefShutOffCams = Set.of(); // Indices of cameras to shut off when PID aligning to the REEF.
        public static final Set<Integer> coralStationShutOffCams = Set.of(); // Indices of cameras to shut off when PID aligning to the CORAL STATION.
        public static final Set<Integer> processorShutOffCams = Set.of(); // Indices of cameras to shut off when PID aligning to the PROCESSOR.

        public static final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(0.2, 0.2, 0.5);
        public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.1, 0.1, 0.2);
    }

    public static class AutoConstants {
        // Used for constraints of pathfinding
        public static final double maxVelocityMPS = 2.0; // Max robot speed, m/s, 4.73

        public static final double maxAccelMPS2 = 1.5; // Max robot accelerations m/s/s

        public static final double maxSpinRadPS = 3.0 * (2.0 * Math.PI); // Max robot angular velocity, radians/s

        public static final double maxSpinAccelRadPS2 = 2.0 * (2.0 * Math.PI); // Max robot angular acceleration, radians/s/s

        public static final double fineTuneMaxTime = 3.0;

        public static final double lidarFineTuneMaxTime = 2.0;

        public static final double pidDistanceTolerance = 0.03;

        public static final double lidarDistanceTolerance = 0.01;
    }

    public static class RobotConstantsMeters {
        // METERS!!!
        // Half of the robot's width, in meters
        public static final double halfWidth = 0.432;
        // Note: Coral intake is to the LEFT of the robot's center (from the robot's perspective)
        public static final double coralArmOffset = 0.154;

        // Distance between front of bumper and lidar
        public static final double lidarBumperDistance = 0.085;

        // Gap between robot and places to score
        public static final double processorSafeDist = 0.25;

        public static final double cageSafeDist = 0.1;

        public static final double reefSafeDist = 0.225;

        public static final double reefAlgaeSafeDist = -reefSafeDist; // Gets added to reefSafeDist

        public static final double coralStationSafeDist = 0.1775;

        public static final double reefDistCorrectionL4 = -0.025;

        // X-offset from middle of the field to middle of the robot at match start
        public static final double startingPointOffset = 1.614176;

        // Horizontal (from robot perspective) translations
        public static final double coralStationCorrection = 0.0508; // Gets added onto coralArmOffset
        
        public static final double reefBranchCorrection = -0.020; // -0.063

        public static final double reefBranchCorrectionL4 = 0.010;
    }

    public static class FieldConstants {
        // Coordinates are in meters, and for the Pose2d class
        // All distances are in meters

        // Reef measurements
        public static final double blueReefCenterX = 4.489323;
        public static final double blueReefCenterY = 4.0259;

        public static final double redReefCenterX = 13.058902;
        public static final double redReefCenterY = 4.0259;

        public static final double reefWallDistance = 0.831723;
        public static final double reefCoralOffset = 0.164;

        // Coral Station
        public static final double coralStationBlueX = 0.851154;
        public static final double coralStationRedX = 16.697198;
        
        public static final double coralStationTopY = 7.39648;
        public static final double coralStationBottomY = 0.65532;

        public static final double coralStationOffset = 0.406;

        // Algae Processor
        public static final double blueProcessorX = 5.987542;
        public static final double blueProcessorY = -0.00381;

        public static final double redProcessorX = 11.560801;
        public static final double redProcessorY = 8.05561;
        
        // Barge and Cages
        public static final double bargeCenterX = 8.774176;
        public static final double bargeCenterY = 4.0259;

        public static final double firstCageOffsetY = 0.980;
        public static final double cageOffsetY = 1.091;
        public static final double initialCageOffsetDifference = cageOffsetY - firstCageOffsetY;
        public static final double cageOffsetX = 0.093; // half of width of cage
    }

    // Backup Spark Maxes:
    //  ----------------------------
    // | LABEL    | CAN ID | STATUS |
    //  ----------------------------
    // | "SM 004" | 30     |        |
    // | "SM 005" | 31     | BROKEN |
    // | "SM 006" | 32     | IN USE |
    //  ----------------------------

    public static class FlywheelConstants {
        public static final int flywheelMotorID = 30; // CAN
        public static final int flywheelMotorFollowerID = 32; //CAN
        public static final int intakeSensorID = 3; // DIO
        public static final double intakeSpeed = 0.5;
        public static final double shootSpeed = -0.5;
        public static final double shootDuration = 0.5;
        public static final double intakeMaxTime = 10.0;
    }

    public static class WristConstants {
        public static final int wristMotorID = 15; // CAN
        public static final int wristEncoderID = 0; // DIO
    
        public static final double intakePosition = 0.14;
        /**L1*/
        public static final double lowShootPosition = 0.26828;
        /**L2/L3*/
        public static final double midShootPosition = 0.2926;
        /**L4*/
        public static final double highShootPosition = 0.28;
        public static final double encoderOffset = 0.09;
    
        public static final double kP = 5.0;
        public static final double kI = 0.1;
        public static final double kD = 0.1;
        
        public static final double maxV = 2.0; // rotations/s/s
        public static final double maxA = 1.0; // rotations/s
    }

    public static class ElevatorConstants{
        public static final int elevatorMotorID = 13; // CAN
        public static final int followMotorID = 14; // CAN
        // public static final int topLimitSwitchID = 1; // DIO
        // public static final int bottomLimitSwitchID = 2; // DIO
        public static final int encoderA = 1; // DIO, Blue
        public static final int encoderB = 2; // DIO, Yellow

        public static final double kP = 6.0; //4.0
        public static final double kI = 0.1;
        public static final double kD = 0.1;

        public static final double kS = 0.0;
        /** If this one is too low, elevator goes slamming down. If too high, elevator goes slamming up. Get this one right. */
        public static final double kG = 0.325; // 0.4225 // Get this one to 3 decimal places of precision, 0.129 is STARTING POINT ONLY!!!
        public static final double kV = 3.4; // 3.6
        public static final double kA = 0.0;

        public static final double maxV = 3.5; // Max velocity, 2.5
        public static final double maxA = 2.5; // Max acceleration

        public static final double elevatorGearRadius = 0.022;
        public static final double encoderPulseDist = (2.0 * Math.PI * elevatorGearRadius) / 1024;

        public static final double level1Position = 0.15;
        public static final double level2Position = 0.3625;
        public static final double level3Position = 0.7827;
        public static final double level4Position = 1.335;
        public static final double intakePosition = 0.21;
        public static final double floorAlgaePosition = 0.0;
        
        public static final double L2AlgaeHeight = 0.60; // 0.608
        public static final double L3AlgaeHeight = 1.0; // 0.956

        public static final double maxHeight = 1.335;
        public static final double minHeight = 0.0;

        public static final double processorHeight = 0.2;

        // Elevator is at goal position if it is this close to the goal position
        public static final double goalRange = 0.025;

        public static final double encoderOffset = 0.049; // Height when starting
    }

    public static class AlgaeIntakeConstants {
        public static final int motorID1 = 15; // CAN
        public static final int motorID2 = 16; // CAN
        public static final int algaeIntakeSensorID = 4; // DIO

        public static final double outSpeed = -1.0; // -
        public static final double inSpeed = 0.5; // - 

        public static final double maxMotorTime = 3.0;
        public static final double maxGroundPickupTime = 20.0;
        public static final double outMotorTime = 0.5;

        public static final double normalSpeed = 0.03;
    }

    public static class ClimberConstants {
        public static final int motorID = 17; // CAN

        public static final double climbSpeed = 0.70;

        public static final double climbRotations = 65.0;
    }

    public static class LEDConstants {
        public static final int pwmChannel = 0; // PWM port

        public static final double normalColor = 0.93; // 0.93 = White

        public static final double autonColor = 0.61; // 0.67 = Gold, 0.61 = Red

        public static final double level1Color = 0.71; // 0.71 = Lawn Green

        public static final double level2Color = 0.89; // 0.89 = Blue Violet

        public static final double level3Color = 0.57; // 0.57 = Hot Pink

        public static final double level4Color = 0.81; // 0.81 = Aqua
    }
}
