package frc.team696.lib.Camera;
/* 
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.net.PortForwarder;
import frc.team696.lib.Logging.PLog;

/**
 *  PhotonVision Camera Interface 
 * */
/* 
public class PhotonVisionCam extends BaseCam {

    public String _name;
    private Transform3d _position;

    private PhotonCamera _camera;
    private PhotonPoseEstimator _estimator;

    public static AprilTagFieldLayout _aprilTagFieldLayout;

    public PhotonVisionCam(String name, Transform3d position) {
        this._name = name;
        this._position = position;

        _camera = new PhotonCamera(_name);
        _estimator = new PhotonPoseEstimator(
            _aprilTagFieldLayout,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            _camera,
            _position
        );
        _estimator.setMultiTagFallbackStrategy(
            PoseStrategy.LOWEST_AMBIGUITY
        );
    }
    
    public Optional<AprilTagResult> getEstimate() {
        Optional<EstimatedRobotPose> oLatestEstimate = _estimator.update();

        if (oLatestEstimate.isEmpty()) return Optional.empty();

        EstimatedRobotPose latestEstimate = oLatestEstimate.get();

        return Optional.of(
            new AprilTagResult(latestEstimate.estimatedPose.toPose2d(), 
                latestEstimate.timestampSeconds, 
                latestEstimate.targetsUsed.get(0).getBestCameraToTarget().getTranslation().getNorm(), 
                latestEstimate.targetsUsed.size(),
                latestEstimate.targetsUsed.get(0).getPoseAmbiguity()));
    }

    static {
        try { 
            _aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
        } catch (Exception e) {
            PLog.fatalException(
                "PhotonVisionCam",
                "Failed to load april tag layout.",
                e
            );
        }

        PortForwarder.add(5800, "photonvision.local", 5800);

        PhotonCamera.setVersionCheckEnabled(false);
    }
}
*/