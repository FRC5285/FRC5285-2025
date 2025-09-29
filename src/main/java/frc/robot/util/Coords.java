package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.Constants.RobotConstantsMeters;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.FieldConstants;

// I hate math. I hate math. I hate math.
public class Coords {
    public final boolean isBlue;
    public final double sideSign; // 1.0 for blue, -1.0 for red, meant as a multiplier
    public final int teamStationNumber;
    public final Pose2d topCoralStationPose;
    public final Pose2d bottomCoralStationPose;
    public final Pose2d processorPose;
    public final Pose2d processorPoseFar;
    public final Pose2d cagePose;
    public final Pose2d cagePoseFar;
    public final Pose2d startPose; // Default starting position
    public final Pose2d[] reefSideCenterLocs;
    public final Rotation2d[] reefCorrespondingAngles;
    public final double[] reefAlgaeHeights;
    public final Pose2d[] startingPositions;

    private final double cos36 = Math.cos(0.2 * Math.PI); // Cosine of 36 deg (coral station)
    private final double sin36 = Math.sin(0.2 * Math.PI); // Sine of 36 deg (coral station)

    public Coords(boolean isBlue) {
        this.isBlue = isBlue;
        this.sideSign = this.isBlue ? 1.0 : -1.0;

        if (DriverStation.getLocation().isPresent()) this.teamStationNumber = DriverStation.getLocation().getAsInt();
        else this.teamStationNumber = 1;

        double proX, proY, proFarX, proFarY, reefCenterX, reefCenterY, coralStationX;
        if (this.isBlue) {
            reefCenterX = FieldConstants.blueReefCenterX;
            reefCenterY = FieldConstants.blueReefCenterY;
            coralStationX = FieldConstants.coralStationBlueX;
            proX = FieldConstants.blueProcessorX;
            proY = FieldConstants.blueProcessorY;
            proFarX = FieldConstants.redProcessorX;
            proFarY = FieldConstants.redProcessorY;
        } else {
            reefCenterX = FieldConstants.redReefCenterX;
            reefCenterY = FieldConstants.redReefCenterY;
            coralStationX = FieldConstants.coralStationRedX;
            proX = FieldConstants.redProcessorX;
            proY = FieldConstants.redProcessorY;
            proFarX = FieldConstants.blueProcessorX;
            proFarY = FieldConstants.blueProcessorY;
        }

        this.processorPose = new Pose2d(
            proX,
            proY + this.sideSign * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.processorSafeDist),
            new Rotation2d(Math.PI + this.sideSign * (Math.PI / 2.0))
        );
        this.processorPoseFar = new Pose2d(
            proFarX,
            proFarY - this.sideSign * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.processorSafeDist),
            new Rotation2d(Math.PI - this.sideSign * (Math.PI / 2.0))
        );

        this.cagePose = new Pose2d(
            FieldConstants.bargeCenterX - this.sideSign * (FieldConstants.cageOffsetX + RobotConstantsMeters.cageSafeDist + RobotConstantsMeters.halfWidth),
            FieldConstants.bargeCenterY + this.sideSign * (FieldConstants.firstCageOffsetY +  FieldConstants.cageOffsetY * (3 - this.teamStationNumber)),
            new Rotation2d(Math.PI/2.0 + this.sideSign * (Math.PI/2.0))
        );
        this.cagePoseFar = new Pose2d(
            FieldConstants.bargeCenterX + this.sideSign * (FieldConstants.cageOffsetX + RobotConstantsMeters.cageSafeDist + RobotConstantsMeters.halfWidth),
            FieldConstants.bargeCenterY + this.sideSign * (FieldConstants.firstCageOffsetY +  FieldConstants.cageOffsetY * (3 - this.teamStationNumber)),
            new Rotation2d(Math.PI/2.0 - this.sideSign * (Math.PI/2.0))
        );

        this.startPose = new Pose2d(
            8.75 - 1.25 * this.sideSign,
            4.0,
            new Rotation2d(Math.PI/2.0 - this.sideSign * (Math.PI/2.0))
        ); // Position is facing towards barge, at center of starting line

        coralStationX += this.sideSign * (this.cos36 * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.coralStationSafeDist));
        this.bottomCoralStationPose = new Pose2d(
            coralStationX - this.cos36 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            FieldConstants.coralStationBottomY + (this.sin36 * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.coralStationSafeDist)) + this.sideSign * this.sin36 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            new Rotation2d(1.5 * Math.PI - this.sideSign * (0.2 * Math.PI))
        );
        this.topCoralStationPose = new Pose2d(
            coralStationX + this.cos36 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            FieldConstants.coralStationTopY - (this.sin36 * (RobotConstantsMeters.halfWidth + RobotConstantsMeters.coralStationSafeDist)) + this.sideSign * this.sin36 * (RobotConstantsMeters.coralArmOffset + RobotConstantsMeters.coralStationCorrection),
            new Rotation2d(0.5 * Math.PI + this.sideSign * (0.2 * Math.PI))
        );

        Pose2d[] spTemp = new Pose2d[7];
        for (int i = 0; i <= 6; i++) {
            spTemp[i] = new Pose2d(
                FieldConstants.bargeCenterX - this.sideSign * RobotConstantsMeters.startingPointOffset,
                FieldConstants.bargeCenterY + this.sideSign * ((3.0 - i) * FieldConstants.cageOffsetY - Math.signum(3.0 - i) * FieldConstants.initialCageOffsetDifference),
                new Rotation2d(Math.PI/2.0 + this.sideSign * (Math.PI/2.0))
            );
        }
        this.startingPositions = spTemp;

        Pose2d[] rscl = new Pose2d[6];
        Rotation2d[] rca = new Rotation2d[6];
        double[] rah = new double[6];
        double a;
        double centerHeight = (ElevatorConstants.L2AlgaeHeight + ElevatorConstants.L3AlgaeHeight) / 2.0;
        double addHeight = Math.abs(ElevatorConstants.L2AlgaeHeight - ElevatorConstants.L3AlgaeHeight) / 2.0;
        for (int i = 0; i < 6; i++) {
            a = i * (Math.PI / 3);
            rca[i] = new Rotation2d((a + Math.PI) % (Math.PI * 2));
            rscl[i] = new Pose2d(
                reefCenterX + Math.cos(a) * (FieldConstants.reefWallDistance + RobotConstantsMeters.reefSafeDist + RobotConstantsMeters.halfWidth),
                reefCenterY + Math.sin(a) * (FieldConstants.reefWallDistance + RobotConstantsMeters.reefSafeDist + RobotConstantsMeters.halfWidth),
                rca[i]
            );
            rah[i] = centerHeight - this.sideSign * addHeight * (i % 2 == 0 ? 1.0 : -1.0);
        }
        this.reefCorrespondingAngles = rca;
        this.reefSideCenterLocs = rscl;
        this.reefAlgaeHeights = rah;
    }

    // gets optimal position to receive coral
    public Pose2d getCoralStationCoords(Pose2d robotPose, boolean goLeft, boolean goRight) {
        // y = 4 is center of field
        boolean goToTop = robotPose.getY() > 4.0;
        return getCoralStationCoords(goToTop, goLeft, goRight);
    }

    public Pose2d getCoralStationCoordsLeftRight(boolean goLeftStation, boolean goLeft, boolean goRight) {
        return getCoralStationCoords(!(goLeftStation ^ this.sideSign == 1.0), goLeft, goRight);
    }

    public Pose2d getCoralStationCoords(boolean goToTop, boolean goLeft, boolean goRight) {
        // if both left and right are pressed down (or none), then choose closest coral station
        Pose2d goToPose = goToTop ? this.topCoralStationPose : this.bottomCoralStationPose;
        if (goLeft == goRight) return goToPose;
        double goingLeft = goLeft ? 1.0 : -1.0;
        double goingTop = goToTop ? 1.0 : -1.0;
        return new Pose2d(
            goToPose.getX() + goingTop * goingLeft * this.cos36 * FieldConstants.coralStationOffset,
            goToPose.getY() + goingLeft * this.sideSign * this.sin36 * FieldConstants.coralStationOffset,
            goToPose.getRotation()
        );
    }

    public Pose2d getClosestProcessor(Pose2d robotPose) {
        if (this.sideSign == 1.0) {
            return robotPose.getX() > FieldConstants.bargeCenterX ? this.processorPoseFar : this.processorPose;
        } else {
            return robotPose.getX() > FieldConstants.bargeCenterX ? this.processorPose : this.processorPoseFar;
        }
    }

    public Pose2d getClosestCage(Pose2d robotPose) {
        if (this.sideSign == 1.0) {
            return robotPose.getX() > FieldConstants.bargeCenterX ? this.cagePoseFar : this.cagePose;
        } else {
            return robotPose.getX() > FieldConstants.bargeCenterX ? this.cagePose : this.cagePoseFar;
        }
    }

    // returns index of coords of closest reef wall
    public int getReefWallCenterCoordsIndex(Pose2d robotPose) {
        int rtnIndex = 0;
        double dist2 = Double.MAX_VALUE;
        Pose2d p2d;
        for (int i = 0; i < 6; i ++) {
            p2d = this.reefSideCenterLocs[i];
            if (dist2 > Math.hypot(robotPose.getX() - p2d.getX(), robotPose.getY() - p2d.getY())) {
                rtnIndex = i;
                dist2 = Math.hypot(robotPose.getX() - p2d.getX(), robotPose.getY() - p2d.getY());
            }
        }
        return rtnIndex;
    }

    public double getAlgaeHeight(Pose2d robotPose) {
        return this.reefAlgaeHeights[this.getReefWallCenterCoordsIndex(robotPose)];
    }

    // returns coords of center of closest reef wall
    public Pose2d getReefWallCenterCoords(Pose2d robotPose) {
        return this.reefSideCenterLocs[this.getReefWallCenterCoordsIndex(robotPose)];
    }

    // returns optimal algae scoring position
    public Pose2d getReefAlgaeCoords(Pose2d robotPose) {
        Pose2d wall2d = this.getReefWallCenterCoords(robotPose);
        wall2d = new Pose2d( // puts an intake arm in the center
            wall2d.getX() + wall2d.getRotation().getSin() * (RobotConstantsMeters.coralArmOffset),
            wall2d.getY() - wall2d.getRotation().getCos() * (RobotConstantsMeters.coralArmOffset),
            wall2d.getRotation()
        );
        return new Pose2d(
            wall2d.getX() - wall2d.getRotation().getCos() * RobotConstantsMeters.reefAlgaeSafeDist,
            wall2d.getY() - wall2d.getRotation().getSin() * RobotConstantsMeters.reefAlgaeSafeDist,
            wall2d.getRotation()
        );
    }

    /** For use before depositing coral, lets cameras adjust pose */
    public Pose2d preDepositCoralCoords(Pose2d goToPose) {
        return this.preDepositCoralCoords(goToPose, 0.4);
    }

    public Pose2d preDepositCoralCoords(Pose2d goToPose, double dist) {
        return new Pose2d(
            goToPose.getX() - goToPose.getRotation().getCos() * dist,
            goToPose.getY() - goToPose.getRotation().getSin() * dist,
            goToPose.getRotation()
        );
    }

    // returns optimal coral scoring position
    // note: maybe add different positions for scoring on different levels?
    public Pose2d getReefBranchCoords(Pose2d robotPose, double distCorrection, boolean goLeft, boolean goRight, double leftRightCorrection) {
        return getReefBranchCoords(this.getReefWallCenterCoordsIndex(robotPose), distCorrection, goLeft, goRight, leftRightCorrection);
    }

    public Pose2d getReefBranchCoordsAuto(int reefSide, double distCorrection, boolean goLeft, boolean goRight, double leftRightCorrection) {
        return getReefBranchCoords(sideSign == 1.0 ? reefSide : ((reefSide + 3) % 6), distCorrection, goLeft, goRight, leftRightCorrection);
    }

    public Pose2d getReefBranchCoords(int wallIndex, double distCorrection, boolean goLeft, boolean goRight, double leftRightCorrection) {
        Pose2d wall2d = this.reefSideCenterLocs[wallIndex];
        double goingLeft = goLeft ? 1.0 : -1.0;
        // cool math that adds the coral intake offset
        // yes, the sin and cos are intentionally swapped (rotates the direction by 270 deg)
        wall2d = new Pose2d(
            wall2d.getX() + wall2d.getRotation().getSin() * (RobotConstantsMeters.coralArmOffset + leftRightCorrection),
            wall2d.getY() - wall2d.getRotation().getCos() * (RobotConstantsMeters.coralArmOffset + leftRightCorrection),
            wall2d.getRotation()
        );
        wall2d = new Pose2d( // Correction for L4
            wall2d.getX() - wall2d.getRotation().getCos() * distCorrection,
            wall2d.getY() - wall2d.getRotation().getSin() * distCorrection,
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
