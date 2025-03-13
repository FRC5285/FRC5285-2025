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
import frc.robot.Constants.AutoConstants;
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
                Pose2d goToCoords = this.coords.getReefBranchCoords(this.drivetrain.getState().Pose, controller.getLeftBumperButton(), controller.getRightBumperButton());
                return AutoBuilder.pathfindToPose(
                    goToCoords,
                    this.pathfindConstraints,
                    0.0
                )
                .andThen(this.drivetrain.fineTunePID(goToCoords))
                .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
                .andThen(new WaitUntilCommand(() -> wrist.isAtSetpoint()))
                .andThen(flywheel.shootCoral());
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
                .andThen(this.drivetrain.fineTunePID(goToCoords))
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
                .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
                .andThen(algaeIntake.doIntake());
            },
            Set.of(this)
        ); 
    }

    public Command doProcessor(ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake) {
        return new DeferredCommand(
            () -> {
                Pose2d goToPose = this.coords.getClosestProcessor(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToPose,
                    this.pathfindConstraints,
                    0.0
                )
                .alongWith(elevator.goToProcessorPosition()) // Failsafe
                .andThen(this.drivetrain.fineTunePID(goToPose))
                .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
                .andThen(algaeIntake.shootOut());
            },
            Set.of(this)
        );
    }

    public Command doDeepClimb() {
        return new DeferredCommand(() -> {
                Pose2d goToPose = this.coords.getClosestCage(this.drivetrain.getState().Pose);
                return AutoBuilder.pathfindToPose(
                    goToPose,
                    this.pathfindConstraints,
                    0.0
                )
                .andThen(drivetrain.fineTunePID(goToPose));
            },
            Set.of(this)
        );
    }
}
