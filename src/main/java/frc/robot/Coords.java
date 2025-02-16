package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.RobotConstantsMeters;
import frc.robot.Constants.FieldConstants;

// I hate math. I hate math. I hate math.
public class Coords {
    public final boolean isBlue;
    public final double sideSign; // 1.0 for blue, -1.0 for red, meant as a multiplier
    public final int teamStationNumber;
    public final Pose2d topCoralStationPose;
    public final Pose2d bottomCoralStationPose;
    public final Pose2d processorPose;
    public final Pose2d cagePose;
    public final Pose2d[] reefSideCenterLocs;
    public final Rotation2d[] reefCorrespondingAngles;

    // Value of sin(45 deg) or cos(45 deg)
    private final double sc45 = Math.cos(Math.PI / 4.0);

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

        coralStationX += this.sideSign * (this.sc45 * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.coralStationSafeDist));
        this.bottomCoralStationPose = new Pose2d(
            coralStationX - this.sc45 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            FieldConstants.coralStationBottomY + (this.sc45 * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.coralStationSafeDist)) + this.sideSign * this.sc45 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            new Rotation2d(1.5 * Math.PI - this.sideSign * (Math.PI/4.0))
        );
        this.topCoralStationPose = new Pose2d(
            coralStationX + this.sc45 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            FieldConstants.coralStationTopY - (this.sc45 * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.coralStationSafeDist)) + this.sideSign * this.sc45 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            new Rotation2d(0.5 * Math.PI + this.sideSign * (Math.PI/4.0))
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

    // gets optimal position to receive coral
    public Pose2d getCoralStationCoords(Pose2d robotPose, boolean goLeft, boolean goRight) {
        // y = 4 is center of field
        boolean goToTop = robotPose.getY() > 4.0;
        // if both left and right are pressed down (or none), then choose closest coral station
        if (goLeft == goRight) return goToTop ? this.topCoralStationPose : this.bottomCoralStationPose;
        double goingLeft = goLeft ? 1.0 : -1.0;
        double goingTop = goToTop ? 1.0 : -1.0;
        return new Pose2d(
            this.bottomCoralStationPose.getX() + goingTop * goingLeft * this.sc45 * FieldConstants.coralStationOffset,
            this.bottomCoralStationPose.getY() + goingLeft * this.sideSign * this.sc45 * FieldConstants.coralStationOffset,
            this.bottomCoralStationPose.getRotation()
        );
    }

    // returns index of coords of closest reef wall
    public int getReefWallCenterCoordsIndex(Pose2d robotPose) {
        int rtnIndex = 0;
        double dist2 = Double.MAX_VALUE;
        Pose2d p2d;
        for (int i = 0; i < 6; i ++) {
            p2d = this.reefSideCenterLocs[i];
            if (dist2 > (Math.pow(robotPose.getX() - p2d.getX(), 2) + Math.pow(robotPose.getY() - p2d.getY(), 2))) {
                rtnIndex = i;
                dist2 = (Math.pow(robotPose.getX() - p2d.getX(), 2) + Math.pow(robotPose.getY() - p2d.getY(), 2));
            }
        }
        return rtnIndex;
    }

    // returns coords of center of closest reef wall
    public Pose2d getReefWallCenterCoords(Pose2d robotPose) {
        return this.reefSideCenterLocs[this.getReefWallCenterCoordsIndex(robotPose)];
    }

    // returns optimal algae scoring position
    public Pose2d getReefAlgaeCoords(Pose2d robotPose) {
        Pose2d wall2d = this.getReefWallCenterCoords(robotPose);
        return new Pose2d(
            wall2d.getX() - wall2d.getRotation().getCos() * RobotConstantsMeters.reefAlgaeSafeDist,
            wall2d.getY() - wall2d.getRotation().getSin() * RobotConstantsMeters.reefAlgaeSafeDist,
            wall2d.getRotation()
        );
    }

    // returns optimal coral scoring position
    // note: maybe add different positions for scoring on different levels?
    public Pose2d getReefBranchCoords(Pose2d robotPose, boolean goLeft, boolean goRight) {
        int wallIndex = this.getReefWallCenterCoordsIndex(robotPose);
        Pose2d wall2d = this.reefSideCenterLocs[wallIndex];
        double goingLeft = goLeft ? 1.0 : -1.0;
        // cool math that adds the coral intake offset
        // yes, the sin and cos are intentionally swapped (rotates the direction by 270 deg)
        wall2d = new Pose2d(
            wall2d.getX() + wall2d.getRotation().getSin() * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.reefBranchCorrection),
            wall2d.getY() - wall2d.getRotation().getCos() * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.reefBranchCorrection),
            wall2d.getRotation()
        );
        if (goLeft == goRight) return wall2d;
        // robot at walls (x) and (x+3) will move in the same direction, if moving left/right along wall from driver perspective
        // walls are numbered counterclockwise regardless of team color, starting at the right-most reef wall as wall 0, going to wall 5
        // (see rca/rscl from the constructor)
        double xMult = wallIndex == 0 || wallIndex == 3 ? 0.0 : wallIndex == 1 || wallIndex == 4 ? -1.0 : 1.0;
        return new Pose2d(
            wall2d.getX() + xMult * goingLeft * this.sideSign * Math.cos(Math.PI / 6.0) * FieldConstants.reefCoralOffset,
            wall2d.getY() + goingLeft * this.sideSign * (xMult != 0.0 ? Math.sin(Math.PI / 6.0) : 1.0) * FieldConstants.reefCoralOffset,
            wall2d.getRotation()
        );
    }
}
