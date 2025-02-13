package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.RobotConstantsMeters;
import frc.robot.Constants.FieldConstants;

public class Coords {
    public final boolean isBlue;
    public final double sideSign;
    public final int teamStationNumber;
    public final Pose2d topCoralStationPose;
    public final Pose2d bottomCoralStationPose;
    public final Pose2d processorPose;
    public final Pose2d cagePose;
    public final Pose2d[] reefSideCenterLocs;
    public final Rotation2d[] reefCorrespondingAngles;

    public Coords(boolean isBlue) {
        this.isBlue = isBlue;
        this.sideSign = this.isBlue ? 1.0 : -1.0;

        if (DriverStation.getLocation().isPresent()) this.teamStationNumber = DriverStation.getLocation().getAsInt();
        else this.teamStationNumber = 1;

        double proX, proY, reefCenterX, reefCenterY, coralStationX;
        if (this.isBlue) {
            reefCenterX = FieldConstants.blueReefCenterX;
            reefCenterY = FieldConstants.blueReefCenterY;
            coralStationX = FieldConstants.coralStationBlueX;
            proX = FieldConstants.blueProcessorX;
            proY = FieldConstants.blueProcessorY;
        } else {
            reefCenterX = FieldConstants.redReefCenterX;
            reefCenterY = FieldConstants.redReefCenterY;
            coralStationX = FieldConstants.coralStationRedX;
            proX = FieldConstants.redProcessorX;
            proY = FieldConstants.redProcessorY;
        }

        this.processorPose = new Pose2d(
            proX,
            proY + this.sideSign * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.processorSafeDist),
            new Rotation2d(Math.PI + this.sideSign * (Math.PI / 2.0))
        );

        this.cagePose = new Pose2d(
            FieldConstants.bargeCenterX - this.sideSign * (FieldConstants.cageOffsetX + RobotConstantsMeters.cageSafeDist + RobotConstantsMeters.halfWidth),
            FieldConstants.bargeCenterY + this.sideSign * (FieldConstants.firstCageOffsetY +  FieldConstants.cageOffsetY * (3 - this.teamStationNumber)),
            new Rotation2d(Math.PI/2.0 - this.sideSign * (Math.PI/2.0))
        );

        coralStationX += this.sideSign * (Math.cos(Math.PI/4.0) * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.coralStationSafeDist));
        this.bottomCoralStationPose = new Pose2d(

        );
        this.topCoralStationPose = new Pose2d(

        );

        Pose2d[] rscl = new Pose2d[6];
        Rotation2d[] rca = new Rotation2d[6];
        double a;
        for (int i = 0; i < 6; i++) {
            a = i * (Math.PI / 3);
            rca[i] = new Rotation2d((a + Math.PI) % (Math.PI * 2));
            rscl[i] = new Pose2d(
                reefCenterX + Math.cos(a) * (FieldConstants.reefWallDistance + RobotConstantsMeters.reefSafeDist + RobotConstantsMeters.halfWidth),
                reefCenterY + Math.sin(a) * (FieldConstants.reefWallDistance + RobotConstantsMeters.reefSafeDist + RobotConstantsMeters.halfWidth),
                rca[i]
            );
        }
        this.reefCorrespondingAngles = rca;
        this.reefSideCenterLocs = rscl;
    }

    public Pose2d getCoralStationCoords(Pose2d robotPose, boolean goLeft, boolean goRight) {
        return new Pose2d();
    }
}
