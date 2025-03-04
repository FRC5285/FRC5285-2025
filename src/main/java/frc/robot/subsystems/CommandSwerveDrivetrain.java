package frc.robot.subsystems;

import java.util.function.Supplier;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;

import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AimbotCommands;

/*
 * Reference:
 * https://github.com/CrossTheRoadElec/Phoenix6-Examples/blob/main/java/SwerveWithPathPlanner/src/main/java/frc/robot/subsystems/CommandSwerveDrivetrain.java
 * 
 * If you want to implement auton with PathPlanner, code is on here
 * Especially add the AutoBuilder stuff
 */

public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    private AimbotCommands abcs;

    private SlewRateLimiter xLimiter = new SlewRateLimiter(OperatorConstants.accelLimit);
    private SlewRateLimiter yLimiter = new SlewRateLimiter(OperatorConstants.accelLimit);
    private SlewRateLimiter rotLimiter = new SlewRateLimiter(OperatorConstants.rotLimit);


    // for Auton
    /** Swerve request to apply during robot-centric path following */
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        abcs = new AimbotCommands(DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() == Alliance.Blue : true);
        // Configures robot settings for Auton
        configureAutoBuilder();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return runOnce(() -> {
            this.xLimiter.reset(0);
            this.yLimiter.reset(0);
            this.rotLimiter.reset(0);
        }).andThen(
            run(() -> this.setControl(requestSupplier.get()))
        );
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

    public void resetSide() {
        DriverStation.getAlliance().ifPresent(allianceColor -> {
            abcs.updateSide(allianceColor == Alliance.Blue);
            setOperatorPerspectiveForward(
                allianceColor == Alliance.Red
                    ? kRedAlliancePerspectiveRotation
                    : kBlueAlliancePerspectiveRotation
            );
            m_hasAppliedOperatorPerspective = true;
        });
        this.resetPose(abcs.getStartLoc()); // sets robot position to middle of starting line
    }

    @Override
    public void periodic() {
        // sets operator perspective for odometry
        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            // If alliance color is set, then check if it is red
            // If so, orient to red setting, otherwise, orient to blue setting
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }
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
                () -> (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) && DriverStation.isAutonomous(),
                this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    public Command stopCurrentCommand() {
        return runOnce(() -> {});
    }

    public double getAlgaeHeight() {
        return abcs.getAlgaeHeight(this.getState().Pose);
    }

    // Don't change these, change them in AimbotCommands
    public Command depositReefBranch(FlywheelSubsystem flywheel, XboxController controller, ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new DeferredCommand(
            () -> {
                return abcs.depositReefBranch(this.getState().Pose, controller, flywheel, elevator, wrist);
            },
            Set.of(this)
        );
    }

    public Command collectCoralStation(FlywheelSubsystem flywheel, XboxController controller, ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new DeferredCommand(
            () -> {
                return abcs.collectCoralStation(this.getState().Pose, controller, flywheel, elevator, wrist);
            },
            Set.of(this)
        );
    }

    public Command collectAlgaeFromReef(ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake) {
        return new DeferredCommand(
            () -> {
                return abcs.collectAlgaeFromReef(this.getState().Pose, elevator, algaeIntake);
            },
            Set.of(this)
        );
    }

    public Command doProcessor(ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake) {
        return new DeferredCommand(
            () -> {
                return abcs.doProcessor(this.getState().Pose, elevator, algaeIntake);
            },
            Set.of(this)
        );
    }

    public Command doDeepClimb() {
        return new DeferredCommand(
            () -> {
                return abcs.doDeepClimb(this.getState().Pose);
            },
            Set.of(this)
        );
    }
}
