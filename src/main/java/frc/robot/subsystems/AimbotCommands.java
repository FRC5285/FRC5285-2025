package frc.robot.subsystems;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
    private PathConstraints pathfindConstraints = new PathConstraints(AutoConstants.maxVelocityMPS, AutoConstants.maxAccelMPS2, AutoConstants.maxSpinRadPS, AutoConstants.maxSpinAccelRadPS2);

    public AimbotCommands(CommandSwerveDrivetrain drivetrain, boolean isBlue) {
        this.drivetrain = drivetrain;
        this.coords = new Coords(isBlue);
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

    public Command depositReefBranch(XboxController controller, FlywheelSubsystem flywheel, ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefBranchCoords(this.drivetrain.getState().Pose, elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? 0.1048 : 0.0, controller.getLeftBumperButton(), controller.getRightBumperButton(), elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? -0.005 : RobotConstantsMeters.reefBranchCorrection);
                double[] coordsArr = { goToCoords.getX(), goToCoords.getY(), goToCoords.getRotation().getDegrees()};
                SmartDashboard.putNumberArray("Aimbot Target", coordsArr);
                return AutoBuilder.pathfindToPose(
                    this.coords.preDepositCoralCoords(this.drivetrain.getState().Pose),
                    this.pathfindConstraints,
                    0.0
                )
                .andThen(new WaitCommand(1.0))
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.REEF))
                .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
                .andThen(new WaitUntilCommand(() -> wrist.isAtSetpoint()))
                .andThen(new WaitCommand(0.5))
                .andThen(flywheel.shootCoral(elevator.goingToHeight == elevatorLastSelectedHeight.FOUR ? 1.0 : elevator.goingToHeight == elevatorLastSelectedHeight.ONE ? 0.2 : 0.5));
            },
            Set.of(this)
        );
    }

    public Command collectCoralStation(XboxController controller, FlywheelSubsystem flywheel, ElevatorSubsystem elevator, WristSubsystem wrist) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getCoralStationCoords(this.drivetrain.getState().Pose, controller.getLeftBumperButton(), controller.getRightBumperButton());
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                .alongWith(elevator.goToIntakePosition()) // Failsafe
                .alongWith(wrist.goToIntakePosition()) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.CORALSTATION))
                .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
                .andThen(new WaitUntilCommand(() -> wrist.isAtSetpoint()))
                .andThen(flywheel.intakeCoral());
            },
            Set.of(this)
        );
    }

    public Command collectAlgaeFromReef(ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getReefAlgaeCoords(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                .alongWith(elevator.goToPosition(() -> getAlgaeHeight())) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.REEF))
                .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
                .andThen(algaeIntake.doIntake());
            },
            Set.of(this)
        ); 
    }

    public Command doProcessor(ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake) {
        return new DeferredCommand(
            () -> {
                Pose2d goToCoords = this.coords.getClosestProcessor(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                .alongWith(elevator.goToProcessorPosition()) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToCoords, DrivetrainAligningTo.PROCESSOR))
                .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
                .andThen(algaeIntake.shootOut());
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
}
