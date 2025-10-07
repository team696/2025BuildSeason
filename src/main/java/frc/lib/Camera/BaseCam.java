package frc.team696.lib.Camera;

import java.util.Optional;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import frc.team696.lib.Logging.BackupLogger;
import frc.team696.lib.Logging.PLog;

/**
 * Base Camera Object for handling how cameras will generally be handled
 * 
 * <p>
 * Extend to add functionality for a specific camera
 * 
 * @see LimeLightCam.java
 * @see PhotonVisionCam.java
 */
public abstract class BaseCam {
  public class AprilTagResult {
    public Pose2d pose;
    /*
     * Time Should Be In Current Time, Same as Phoenix6 Swerve
     */
    public double time;

    public double distToTag;
    public int tagCount;

    public double ambiguity;

    public AprilTagResult(Pose2d pose, double time, double distToTag, int tagCount, double ambiguity) {
      this.pose = pose;
      this.time = time;
      this.distToTag = distToTag;
      this.tagCount = tagCount;
      this.ambiguity = ambiguity;
    }
  }

  public abstract Optional<AprilTagResult> getEstimate();

  Vector<N3> stdDeviations = VecBuilder.fill(0.7, 0.7, 2);

  public void setStdDeviations(double x, double y, double r) {
    stdDeviations = VecBuilder.fill(x, y, r);
  }

  @FunctionalInterface
  public static interface addVisionEstimate {
    void accept(Pose2d p, double d, Vector<N3> v);
  }

  @FunctionalInterface
  public static interface acceptEstimate {
    boolean test(AprilTagResult latestResult);
  }

  // eventually switch this to taking in a addVisionEstimate
  public boolean addVisionEstimate(addVisionEstimate addVisionMeasurement, acceptEstimate checkEstimation) {
    Optional<AprilTagResult> oEstimation = this.getEstimate();

    if (oEstimation.isPresent()) {
      AprilTagResult estimation = oEstimation.get();
      try {
        if (!checkEstimation.test(estimation)) {
          BackupLogger.addToQueue("696/Vision/Rejected Pose", estimation.pose);
          return false;
        } else {
          BackupLogger.addToQueue("696/Vision/Accepted Pose", estimation.pose);
        }
      } catch (Exception e) {
        PLog.fatalException("Camera", e.getMessage(), e);
      }
      addVisionMeasurement.accept(
          estimation.pose,
          estimation.time,
          stdDeviations);
      return true;
    }
    return false;
  }

  public synchronized boolean addVisionEstimate(addVisionEstimate addVisionMeasurement) {
    return addVisionEstimate(addVisionMeasurement, (latestResult) -> {
      return true;
    });
  }
}
