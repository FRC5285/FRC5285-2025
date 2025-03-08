package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;


// These links were very helpful:
// https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
public class AprilTagCams extends SubsystemBase {

    private Vision[] camerasArr;
    private CommandSwerveDrivetrain drivetrain;

    public AprilTagCams(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        // Creates array of apriltag pose estimation cameras
        camerasArr = new Vision[VisionConstants.numCameras];
        // Initializes cameras with names and position offsets
        for (int i = 0; i < VisionConstants.numCameras; i ++)
            camerasArr[i] = new Vision(VisionConstants.cameraNames[i], VisionConstants.cameraOffsets[i]);
    }

    @Override
    public void periodic() {
        this.updateEstimatedPose();
    }

    public void updateEstimatedPose() {
        for (Vision camera: this.camerasArr) {
            var visionEst = camera.getEstimatedGlobalPose();
            visionEst.ifPresent(est -> {
                // Change our trust in the measurement based on the tags we can see
                this.drivetrain.addVisionMeasurement(
                // Note: Utils.fpgaToCurrentTime() is required according to the *Phoenix 6* docs/examples
                est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), camera.getEstimationStdDevs());
            });
        }
    }
}