package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Set;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain.DrivetrainAligningTo;


// These links were very helpful:
// https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
public class AprilTagCams extends SubsystemBase {

    private Vision[] camerasArr;
    private CommandSwerveDrivetrain drivetrain;
    private HashMap<DrivetrainAligningTo, Set<Integer>> camsOffHashmap = new HashMap<>();

    public AprilTagCams(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;

        camsOffHashmap.put(DrivetrainAligningTo.NOTHING, VisionConstants.normalShutOffCams);
        camsOffHashmap.put(DrivetrainAligningTo.REEF, VisionConstants.reefShutOffCams);
        camsOffHashmap.put(DrivetrainAligningTo.CORALSTATION, VisionConstants.coralStationShutOffCams);
        camsOffHashmap.put(DrivetrainAligningTo.PROCESSOR, VisionConstants.processorShutOffCams);
        camsOffHashmap.put(DrivetrainAligningTo.BARGE, VisionConstants.normalShutOffCams);

        // Creates array of apriltag pose estimation cameras
        camerasArr = new Vision[VisionConstants.numCameras];
        // Initializes cameras with names and position offsets
        for (int i = 0; i < VisionConstants.numCameras; i ++)
            camerasArr[i] = new Vision(VisionConstants.cameraNames[i], VisionConstants.cameraOffsets[i], i);
    }

    @Override
    public void periodic() {
        this.updateEstimatedPose();
    }

    public void updateEstimatedPose() {
        for (Vision camera: this.camerasArr) {
            var visionEst = camera.getEstimatedGlobalPose();
            if (this.camsOffHashmap.get(this.drivetrain.thingAligningTo).contains(camera.camInd)) continue;
            visionEst.ifPresent(est -> {
                // Change our trust in the measurement based on the tags we can see
                this.drivetrain.addVisionMeasurement(
                // Note: Utils.fpgaToCurrentTime() is required according to the *Phoenix 6* docs/examples
                est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(est.timestampSeconds), camera.getEstimationStdDevs());
            });
        }
    }
}