package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.Coords;

public class AimbotCommands {

    private Coords coords;
    private PathConstraints pathfindConstraints = new PathConstraints(AutoConstants.maxVelocityMPS, AutoConstants.maxAccelMPS2, AutoConstants.maxSpinRadPS, AutoConstants.maxSpinAccelRadPS2);

    public AimbotCommands(boolean isBlue) {
        coords = new Coords(isBlue);
    }

    public void updateSide(boolean isBlue) {
        coords = new Coords(isBlue);
    }

    // All of these need a lot more work
    // just basic commands now, chain them together later
    public Command depositReefBranch(Pose2d robotPose, XboxController controller, FlywheelSubsystem flywheel) {
        // Todo: Add checks to make sure elevator and wrist are in position
        return AutoBuilder.pathfindToPose(
            this.coords.getReefBranchCoords(robotPose, controller.getLeftBumperButtonPressed(), controller.getRightBumperButtonPressed()),
            this.pathfindConstraints,
            0.0
        ).andThen(flywheel.shootCoral());
    }

    public Command collectCoralStation(Pose2d robotPose, XboxController controller, FlywheelSubsystem flywheel) {
        // Todo: Add checks to make sure elevator and wrist are in position
        return AutoBuilder.pathfindToPose(
            this.coords.getCoralStationCoords(robotPose, controller.getLeftBumperButtonPressed(), controller.getRightBumperButtonPressed()),
            this.pathfindConstraints,
            0.0
        ).andThen(flywheel.intakeCoral());
    }

    public Command collectAlgaeFromReef(Pose2d robotPose) {
        return AutoBuilder.pathfindToPose(
            this.coords.getReefAlgaeCoords(robotPose),
            this.pathfindConstraints,
            0.0
        );
    }

    public Command doProcessor() {
        return AutoBuilder.pathfindToPose(
            this.coords.processorPose,
            this.pathfindConstraints,
            0.0
        );
    }

    public Command doDeepClimb() {
        return AutoBuilder.pathfindToPose(
            this.coords.cagePose,
            this.pathfindConstraints,
            0.0
        );
    }
}
