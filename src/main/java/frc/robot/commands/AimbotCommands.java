package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.AlgaeIntakeSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.WristSubsystem;
import frc.robot.util.Coords;

public class AimbotCommands {

    private Coords coords;
    private PathConstraints pathfindConstraints = new PathConstraints(AutoConstants.maxVelocityMPS, AutoConstants.maxAccelMPS2, AutoConstants.maxSpinRadPS, AutoConstants.maxSpinAccelRadPS2);

    public AimbotCommands(boolean isBlue) {
        coords = new Coords(isBlue);
    }

    public void updateSide(boolean isBlue) {
        coords = new Coords(isBlue);
    }

    public Pose2d getStartLoc() {
        return coords.startPose;
    }

    public double getAlgaeHeight(Pose2d robotPose) {
        return coords.getAlgaeHeight(robotPose);
    }

    // All of these need a lot more work
    // just basic commands now, chain them together later
    public Command depositReefBranch(Pose2d robotPose, XboxController controller, FlywheelSubsystem flywheel, ElevatorSubsystem elevator, WristSubsystem wrist) {
        return AutoBuilder.pathfindToPose(
            this.coords.getReefBranchCoords(robotPose, controller.getLeftBumperButton(), controller.getRightBumperButton()),
            this.pathfindConstraints,
            0.0
        )
        .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
        .andThen(new WaitUntilCommand(() -> wrist.isAtSetpoint()))
        .andThen(flywheel.shootCoral());
    }

    public Command collectCoralStation(Pose2d robotPose, XboxController controller, FlywheelSubsystem flywheel, ElevatorSubsystem elevator, WristSubsystem wrist) {
        return AutoBuilder.pathfindToPose(
            this.coords.getCoralStationCoords(robotPose, controller.getLeftBumperButton(), controller.getRightBumperButton()),
            this.pathfindConstraints,
            0.0
        )
        .alongWith(elevator.goToIntakePosition()) // Failsafe
        .alongWith(wrist.goToIntakePosition()) // Failsafe
        .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
        .andThen(new WaitUntilCommand(() -> wrist.isAtSetpoint()))
        .andThen(flywheel.intakeCoral());
    }

    public Command collectAlgaeFromReef(Pose2d robotPose, ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake) {
        return AutoBuilder.pathfindToPose(
            this.coords.getReefAlgaeCoords(robotPose),
            this.pathfindConstraints,
            0.0
        )
        .alongWith(elevator.goToPosition(() -> getAlgaeHeight(robotPose))) // Failsafe
        .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
        .andThen(algaeIntake.doIntake());
    }

    public Command doProcessor(Pose2d robotPose, ElevatorSubsystem elevator, AlgaeIntakeSubsystem algaeIntake) {
        return AutoBuilder.pathfindToPose(
            this.coords.getClosestProcessor(robotPose),
            this.pathfindConstraints,
            0.0
        )
        .alongWith(elevator.goToProcessorPosition()) // Failsafe
        .andThen(new WaitUntilCommand(() -> elevator.reachedGoal()))
        .andThen(algaeIntake.shootOut());
    }

    public Command doDeepClimb(Pose2d robotPose) {
        return AutoBuilder.pathfindToPose( // change to pathfind and then do path
            this.coords.getClosestCage(robotPose),
            this.pathfindConstraints,
            0.0
        );
    }
}
