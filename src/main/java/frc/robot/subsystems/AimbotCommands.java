package frc.robot.subsystems;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
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
    private AlgaeIntakeSubsystem algaeIntake;
    private PathConstraints pathfindConstraints = new PathConstraints(AutoConstants.maxVelocityMPS, AutoConstants.maxAccelMPS2, AutoConstants.maxSpinRadPS, AutoConstants.maxSpinAccelRadPS2);

    public AimbotCommands(CommandSwerveDrivetrain drivetrain, boolean isBlue, FlywheelSubsystem flywheelSubsystem, ElevatorSubsystem elevatorSubsystem, WristSubsystem wristSubsystem, AlgaeIntakeSubsystem algaeIntakeSubsystem) {
        this.drivetrain = drivetrain;
        this.coords = new Coords(isBlue);
        this.flywheel = flywheelSubsystem;
        this.elevator = elevatorSubsystem;
        this.wrist = wristSubsystem;
        this.algaeIntake = algaeIntakeSubsystem;
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
            Set.of(this)
        );
    }

    public Command depositReefBranch(int goToSide, boolean goLeft) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefBranchCoords(goToSide, this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? 0.000 : RobotConstantsMeters.reefDistCorrectionL4, goLeft, !goLeft, this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? RobotConstantsMeters.reefBranchCorrectionL4 : RobotConstantsMeters.reefBranchCorrection);
                return depositReefBranch(goToCoords);
            },
            Set.of(this)
        );
    }

    public Command depositReefBranch(Pose2d goToCoords) {
        return AutoBuilder.pathfindToPose(
            this.coords.preDepositCoralCoords(goToCoords),
            this.pathfindConstraints,
            0.0
        )
        .andThen(this.elevator.goToPosition(this.elevator.goingToHeight))
        .andThen(new WaitCommand(0.5))
        .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.REEF))
        .andThen(this.elevator.goToPosition(this.elevator.goingToHeight))
        .andThen(this.wrist.goToHighShootPosition().onlyIf(() -> this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR))
        .andThen(new WaitUntilCommand(() -> this.elevator.reachedGoal()))
        .andThen(new WaitUntilCommand(() -> this.wrist.isAtSetpoint()))
        .andThen(new WaitCommand(0.5))
        .andThen(this.flywheel.shootCoral(this.elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? 1.0 : this.elevator.goingToHeight == elevatorLastSelectedHeight.ONE ? 0.2 : 0.5));
    }

    public Command collectCoralStation(XboxController controller) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getCoralStationCoords(this.drivetrain.getState().Pose, controller.getLeftBumperButton(), controller.getRightBumperButton());
                return collectCoralStation(goToCoords);
            },
            Set.of(this)
        );
    }

    public Command collectCoralStation(boolean goLeft, boolean moveLeft, boolean moveRight) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getCoralStationCoordsLeftRight(goLeft, moveLeft, moveRight);
                return collectCoralStation(goToCoords);
            },
            Set.of(this)
        );
    }

    public Command collectCoralStation(Pose2d goToCoords) {
        return AutoBuilder.pathfindToPose(
            goToCoords,
            this.pathfindConstraints,
            0.0
        )
        .alongWith(elevator.goToIntakePosition()) // Failsafe
        .alongWith(wrist.goToIntakePosition()) // Failsafe
        .andThen(this.flywheel.intakeCoral())
        .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.CORALSTATION));
    }

    public Command collectAlgaeFromReef() {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefAlgaeCoords(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                // .alongWith(this.elevator.goToPosition(() -> getAlgaeHeight())) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.REEF))
                // .andThen(new WaitUntilCommand(() -> this.elevator.reachedGoal()))
                .andThen(this.algaeIntake.doIntake());
            },
            Set.of(this)
        ); 
    }

    public Command doProcessor() {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getClosestProcessor(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                .alongWith(this.elevator.goToProcessorPosition()) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.PROCESSOR))
                .andThen(new WaitUntilCommand(() -> this.elevator.reachedGoal()))
                .andThen(this.algaeIntake.shootOut());
            },
            Set.of(this)
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
                .andThen(drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.BARGE));
            },
            Set.of(this)
        );
    }
    
    public Command stopCurrentCommand() {
        return runOnce(() -> {});
    }
}
