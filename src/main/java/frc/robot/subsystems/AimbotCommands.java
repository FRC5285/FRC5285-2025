package frc.robot.subsystems;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.RobotConstantsMeters;
import frc.robot.subsystems.CommandSwerveDrivetrain.DrivetrainAligningTo;
import frc.robot.subsystems.ElevatorSubsystem.elevatorLastSelectedHeight;
import frc.robot.util.Coords;

public class AimbotCommands extends SubsystemBase {

    private Coords coords;
    private CommandSwerveDrivetrain drivetrain;
    private FlywheelSubsystem flywheel;
    private ElevatorSubsystem elevator;
    private WristSubsystem wrist;
    private PathConstraints pathfindConstraints = new PathConstraints(AutoConstants.maxVelocityMPS, AutoConstants.maxAccelMPS2, AutoConstants.maxSpinRadPS, AutoConstants.maxSpinAccelRadPS2);

    public AimbotCommands(CommandSwerveDrivetrain drivetrain, boolean isBlue, FlywheelSubsystem flywheelSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem) {
        this.drivetrain = drivetrain;
        this.coords = new Coords(isBlue);
        this.flywheel = flywheelSubsystem;
        this.elevator = elevatorSubsystem;
        this.wrist = wristSubsystem;
        SendableRegistry.add(this, "Aimbot");
        SmartDashboard.putData(this);
    }

    public void updateSide(boolean isBlue) {
        coords = new Coords(isBlue);
    }

    public Pose2d getStartLoc() {
        return coords.startPose;
    }

    public double getAlgaeHeight() {
        return coords.getAlgaeHeight(this.drivetrain.getState().Pose);
    }

    public Pose2d getStartPosition(int positionNum) {
        return coords.startingPositions[positionNum];
    }

    public Command depositReefBranch(XboxController controller) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefBranchCoords(this.drivetrain.getState().Pose, this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? 0.000 : RobotConstantsMeters.reefDistCorrectionL4, controller.getLeftBumperButton(), controller.getRightBumperButton(), this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? RobotConstantsMeters.reefBranchCorrectionL4 : RobotConstantsMeters.reefBranchCorrection);
                return depositReefBranch(goToCoords);
            },
            Set.of(this, this.drivetrain, this.flywheel)
        );
    }

    public Command depositReefBranch(int goToSide, boolean goLeft) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefBranchCoordsAuto(goToSide, this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? 0.000 : RobotConstantsMeters.reefDistCorrectionL4, goLeft, !goLeft, this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? RobotConstantsMeters.reefBranchCorrectionL4 : RobotConstantsMeters.reefBranchCorrection);
                return depositReefBranch(goToCoords);
            },
            Set.of(this, this.drivetrain, this.flywheel)
        );
    }

    public SequentialCommandGroup depositReefBranch(Pose2d goToCoords) {
        return AutoBuilder.pathfindToPose(
            this.coords.preDepositCoralCoords(goToCoords),
            this.pathfindConstraints,
            0.0
        )
        .andThen(this.elevator.goToPosition(this.elevator.goingToHeight))
        .andThen(this.wrist.goToHighShootPosition().onlyIf(() -> this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR))
        .andThen(new WaitCommand(0.25))
        .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.REEF, this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? RobotConstantsMeters.reefSafeDist + RobotConstantsMeters.reefDistCorrectionL4 : RobotConstantsMeters.reefSafeDist, true))
        .andThen(this.elevator.goToPosition(this.elevator.goingToHeight))
        .andThen(new WaitUntilCommand(() -> this.elevator.reachedGoal()))
        .andThen(new WaitUntilCommand(() -> this.wrist.isAtSetpoint()))
        // .andThen(new WaitCommand(0.0))
        .andThen(this.flywheel.shootCoral(this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? -1.0 : this.elevator.goingToHeight == elevatorLastSelectedHeight.ONE ? -0.2 : -0.5))
        .andThen(runOnce(() -> {this.getCurrentCommand().cancel();}));
    }

    public Command collectCoralStation(XboxController controller) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getCoralStationCoords(this.drivetrain.getState().Pose, controller.getLeftBumperButton(), controller.getRightBumperButton());
                return collectCoralStation(goToCoords);
            },
            Set.of(this, this.drivetrain, this.flywheel)
        );
    }

    public Command collectCoralStation(boolean goLeft, boolean moveLeft, boolean moveRight) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getCoralStationCoordsLeftRight(goLeft, moveLeft, moveRight);
                return collectCoralStation(goToCoords);
            },
            Set.of(this, this.drivetrain, this.flywheel)
        );
    }

    public SequentialCommandGroup collectCoralStation(Pose2d goToCoords) {
        return AutoBuilder.pathfindToPose(
            this.coords.preDepositCoralCoords(goToCoords, 0.2),
            this.pathfindConstraints,
            0.0
        )
        .alongWith(elevator.goToIntakePosition()) // Failsafe
        .alongWith(wrist.goToIntakePosition()) // Failsafe
        .andThen(this.flywheel.stopIntake())
        .andThen(this.flywheel.intakeCoral()
            .alongWith(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.CORALSTATION, RobotConstantsMeters.coralStationSafeDist, false)))
        .andThen(runOnce(() -> {this.getCurrentCommand().cancel();}));
    }

    public Command collectAlgaeFromReef(AlgaeIntakeSubsystem algaeIntake) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefAlgaeCoords(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                // .alongWith(this.elevator.goToPosition(() -> getAlgaeHeight())) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.REEF, RobotConstantsMeters.reefAlgaeSafeDist, true))
                // .andThen(new WaitUntilCommand(() -> this.elevator.reachedGoal()))
                .andThen(algaeIntake.doIntake());
            },
            Set.of(this, this.drivetrain)
        );
    }

    public Command knockAlgaeFromReef() {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefAlgaeCoords(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    this.coords.preDepositCoralCoords(goToCoords),
                    this.pathfindConstraints,
                    0.0
                )
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.REEF, -1, true)
                    .alongWith(this.wrist.goToAlgaePosition())
                    .alongWith(this.elevator.goToPosition(() -> this.coords.getAlgaeHeight(goToCoords))))
                .andThen(this.elevator.elevatorUpAlgae())
                .andThen(new WaitUntilCommand(() -> this.elevator.reachedGoal()))
                .andThen(this.wrist.goToAlgaePosition2())
                .andThen(new WaitCommand(0.5))
                .andThen(this.drivetrain.moveBack());
            },
            Set.of(this, this.drivetrain, this.elevator, this.wrist)
        );
    }

    public Command doProcessor(AlgaeIntakeSubsystem algaeIntake) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getClosestProcessor(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                .alongWith(this.elevator.goToProcessorPosition()) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.PROCESSOR, -1.0, true))
                .andThen(new WaitUntilCommand(() -> this.elevator.reachedGoal()))
                .andThen(algaeIntake.shootOut());
            },
            Set.of(this, this.drivetrain)
        );
    }

    public Command doDeepClimb() {
        return new DeferredCommand(() -> {
                Pose2d goToCoords = this.coords.getClosestCage(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                .andThen(drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.BARGE, -1.0, true));
            },
            Set.of(this, this.drivetrain)
        );
    }
    
    public Command stopCurrentCommand() {
        return runOnce(() -> {});
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addBooleanProperty("Command Finished", () -> {
            if (this.getCurrentCommand() != null) return this.getCurrentCommand().isFinished();
            return true;
        }, null);
    }
}
