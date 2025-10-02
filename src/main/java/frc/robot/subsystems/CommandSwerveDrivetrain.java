package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.revrobotics.Rev2mDistanceSensor;
import com.revrobotics.Rev2mDistanceSensor.Port;
import com.revrobotics.Rev2mDistanceSensor.RangeProfile;
import com.revrobotics.Rev2mDistanceSensor.Unit;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.RobotConstantsMeters;

/*
 * Reference:
 * https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
 * 
 * If you want to implement auton with PathPlanner, code is on here
 * Especially add the AutoBuilder stuff
 */

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem, Sendable {
    private final Field2d field2D = new Field2d();

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private Rev2mDistanceSensor lidarSensor = new Rev2mDistanceSensor(Port.kOnboard, Unit.kMillimeters, RangeProfile.kHighSpeed);
    private double lidarGoal = 0.0;
    private boolean lidarOn = false;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(OperatorConstants.accelLimit);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(OperatorConstants.accelLimit);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(OperatorConstants.rotLimit);

    private final SwerveRequest.FieldCentric drivePID = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.SwerveDriveBrake swerveBrake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.RobotCentric lidarDrive = new SwerveRequest.RobotCentric();
    private ProfiledPIDController xPID = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(AutoConstants.maxVelocityMPS, AutoConstants.maxAccelMPS2));
    private ProfiledPIDController yPID = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(AutoConstants.maxVelocityMPS, AutoConstants.maxAccelMPS2));
    private ProfiledPIDController rPID = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(AutoConstants.maxSpinRadPS, AutoConstants.maxSpinAccelRadPS2));
    private double invertMult = 1.0;

    private ProfiledPIDController lidarPID = new ProfiledPIDController(4.0, 0.0, 0.0, new TrapezoidProfile.Constraints(AutoConstants.lidarMaxVelocityMPS, AutoConstants.lidarMaxAccelMPS2));

    public DrivetrainAligningTo thingAligningTo = DrivetrainAligningTo.NOTHING;

    // for Auton
    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        // Configures robot settings for Auton
        configureAutoBuilder();
        this.xPID.setTolerance(AutoConstants.pidDistanceTolerance);
        this.yPID.setTolerance(AutoConstants.pidDistanceTolerance);
        this.lidarPID.setTolerance(AutoConstants.lidarDistanceTolerance);

        this.lidarSensor.setAutomaticMode(true);

        SendableRegistry.add(this, "Drivetrain");
        SmartDashboard.putData(this);
        SmartDashboard.putData("Field", field2D);
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return runOnce(() -> {
            this.xLimiter.reset(0);
            this.yLimiter.reset(0);
            this.rotLimiter.reset(0);
            this.thingAligningTo = DrivetrainAligningTo.NOTHING;
        }).andThen(
            run(() -> this.setControl(requestSupplier.get()))
        );
    }

    /**
     * Fine tunes the robot alignment using PID
     * 
     * @param goHere where to go to
     * @param whatAligningTo what to go to
     * @param distFromObject distance from bumper to object (set as -1 if not using lidar)
     * @param doNormalPid use the position PID
     * @return the command to fine tune with PID
     */
    public Command fineTunePID(Pose2d goHere, DrivetrainAligningTo whatAligningTo, double distFromObject, boolean doNormalPid) {
        return this.fineTunePID(goHere, whatAligningTo, distFromObject, doNormalPid, AutoConstants.lidarFineTuneMaxTime, AutoConstants.lidarDistanceTolerance);
    }

    /**
     * Fine tunes the robot alignment using PID
     * 
     * @param goHere where to go to
     * @param whatAligningTo what to go to
     * @param distFromObject distance from bumper to object (set as -1 if not using lidar)
     * @param doNormalPid use the position PID
     * @param lidarTime how long to use lidar
     * @return the command to fine tune with PID
     */
    public Command fineTunePID(Pose2d goHere, DrivetrainAligningTo whatAligningTo, double distFromObject, boolean doNormalPid, double lidarTime, double lidarTolerance) {
        return runOnce(() -> {
            this.xPID.reset(this.getState().Pose.getX());
            this.yPID.reset(this.getState().Pose.getY());
            this.rPID.reset(this.getState().Pose.getRotation().getRadians());
            this.rPID.enableContinuousInput(0.0, 2 * Math.PI);
            this.xPID.setGoal(goHere.getX());
            this.yPID.setGoal(goHere.getY());
            this.rPID.setGoal(goHere.getRotation().getRadians());
            this.thingAligningTo = whatAligningTo;
        })
        .andThen(
            run(() -> {
                this.setControl(
                    drivePID.withVelocityX(this.xPID.calculate(this.getState().Pose.getX()) * invertMult)
                    .withVelocityY(this.yPID.calculate(this.getState().Pose.getY()) * invertMult)
                    .withRotationalRate(this.rPID.calculate(this.getState().Pose.getRotation().getRadians()))
                );
            })
            .until(() -> this.xPID.atGoal() && this.yPID.atGoal() && this.rPID.atGoal())
            .withTimeout(AutoConstants.fineTuneMaxTime)
            .onlyIf(() -> doNormalPid)
        )
        .andThen(
            runOnce(() -> {
                this.lidarPID.reset(this.getLidarMeters());
                this.lidarPID.setTolerance(lidarTolerance);
                this.lidarPID.setGoal(distFromObject + RobotConstantsMeters.lidarBumperDistance);
                this.lidarGoal = distFromObject + RobotConstantsMeters.lidarBumperDistance;
                this.lidarOn = true;
            })
        )
        .andThen(
            run(() -> {
                this.setControl(
                    lidarDrive.withVelocityX(-this.lidarPID.calculate(this.getLidarMeters()))
                    .withVelocityY(0.0)
                    .withRotationalRate(0.0)
                );
            })
            .onlyIf(() -> distFromObject >= 0.0 && this.getLidarMeters() >= RobotConstantsMeters.lidarBumperDistance)
            .until(() -> this.getLidarMeters() < RobotConstantsMeters.lidarBumperDistance || this.lidarPID.atGoal())
            .withTimeout(lidarTime)
        )
        .andThen(() -> {
            // this.setControl(this.swerveBrake);
            this.setControl(lidarDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            this.lidarOn = false;
        });
    }

    public Command moveBack() {
        return runOnce(() -> {
            this.setControl(
                lidarDrive.withVelocityX(AutoConstants.algaeMoveBackSpeedMPS).withVelocityY(0).withRotationalRate(0.0)
            );
        })
        .andThen(new WaitCommand(AutoConstants.algaeMoveBackSeconds))
        .andThen(
            runOnce(() -> {
                this.setControl(lidarDrive.withVelocityX(AutoConstants.algaeMoveBackSpeedMPS).withVelocityY(0).withRotationalRate(AutoConstants.algaeRotateSpeed));
            })
        )
        .andThen(new WaitCommand(AutoConstants.algaeRotateSeconds))
        .andThen(
            runOnce(() -> {
                this.setControl(lidarDrive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
            })
        );
    }

    public double getLidarMeters() {
        return lidarSensor.getRange() / 1000.0 - AutoConstants.lidarOffset;
    }

    public double getXVal(double currentVal, double throttleVal) {
        return this.xLimiter.calculate(this.applyThrottle(currentVal, throttleVal));
    }

    public double getYVal(double currentVal, double throttleVal) {
        return this.yLimiter.calculate(this.applyThrottle(currentVal, throttleVal));
    }

    public double getRotVal(double currentVal, double throttleVal) {
        return this.rotLimiter.calculate(this.applyThrottle(currentVal, throttleVal));
    }

    private double applyThrottle(double speedVal, double throttleVal) {
        return speedVal * (OperatorConstants.maxSpeedMultiplier * (1.0 - throttleVal * OperatorConstants.throttleMaxReduction));
    }

    public void resetSide(Pose2d startPose) {
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            this.invertMult = allianceColor == Alliance.Blue ? 1.0 : -1.0;
            setOperatorPerspectiveForward(
                allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        });
        this.resetPose(startPose); // sets robot position to middle of starting line
    }

    @Override
    public void periodic() {
        // sets operator perspective for odometry
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            // If alliance color is set, then check if it is red
            // If so, orient to red setting, otherwise, orient to blue setting
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                this.invertMult = allianceColor == Alliance.Blue ? 1.0 : -1.0;
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
        field2D.setRobotPose(getState().Pose);
    }

    // Auton configurator
    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,   // Supplier of current robot pose
                this::resetPose,         // Consumer for seeding pose against auto
                () -> getState().Speeds, // Supplier of current robot speeds
                // Consumer of ChassisSpeeds and feedforwards to drive the robot
                (speeds, feedforwards) -> setControl(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    // ---------------------
                    // CHANGE THESE LATER!!!
                    // CHANGE THESE LATER!!!
                    // CHANGE THESE LATER!!!
                    // ---------------------
                    // PID constants for translation
                    new PIDConstants(10, 0, 0),
                    // PID constants for rotation
                    new PIDConstants(7, 0, 0)
                ),
                config,
                // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                // If in auton, flips if necessary, otherwise does not flip
                // New: no longer flips during auton because of custom auton picker
                () -> (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red),
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command stopCurrentCommand() {
        return runOnce(() -> {});
    }

    public enum DrivetrainAligningTo {
        NOTHING,
        REEF,
        CORALSTATION,
        PROCESSOR,
        BARGE;
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Lidar Distance", () -> this.getLidarMeters(), null);
        builder.addDoubleProperty("X Velocity", () -> this.getState().Speeds.vxMetersPerSecond, null);
        builder.addDoubleProperty("Y Velocity", () -> this.getState().Speeds.vyMetersPerSecond, null);
        builder.addDoubleProperty("Lidar Goal", () -> this.lidarGoal, null);
        builder.addBooleanProperty("Lidar On", () -> this.lidarOn, null);
    }
}
