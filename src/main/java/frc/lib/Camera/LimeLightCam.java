package frc.team696.lib.Camera;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;

/**
 * LimeLight Camera Interface
 */
public class LimeLightCam extends BaseCam {
  public String name = "";
  public static int LimeLightCount = 0;

  private NetworkTable _ntTable;

  private boolean useMegaTag2 = false;

  public LimeLightCam(String name, int[] TagsToCheck, boolean useMegaTag2) {
    this.name = name;

    if (TagsToCheck.length > 0) {
      LimelightHelpers.SetFiducialIDFiltersOverride(name, TagsToCheck);
    }

    for (int port = 5800; port <= 5809; port++) {
      PortForwarder.add(port + 10 * LimeLightCount, String.format("%s.local", this.name), port);
    }

    _ntTable = NetworkTableInstance.getDefault().getTable(name);

    _ntTable.getEntry("ledMode").setNumber(1); // Example of how to use -> This turns off LEDS on the front

    this.useMegaTag2 = useMegaTag2;

    LimeLightCount++;
  }

  public LimeLightCam(String name) {
    this(name, new int[] {}, false);
  }

  public LimeLightCam(String name, boolean useMegaTag2) {
    this(name, new int[] {}, useMegaTag2);
  }

  public int targetCount() {
    double[] t2d = _ntTable.getEntry("t2d").getDoubleArray(new double[0]);
    if (t2d.length == 17) {
      return (int) t2d[1];
    }
    return 0;
  }

  public boolean hasTargets() {
    return targetCount() > 0;
  }

  public double tX() {
    return -1 * _ntTable.getEntry("tx").getDouble(0);
  }

  public void SetRobotOrientation(Rotation2d curYaw) {
    LimelightHelpers.SetRobotOrientation(name, curYaw.getDegrees(), 0, 0, 0, 0, 0);
  }

  public Optional<AprilTagResult> getEstimate() {
    LimelightHelpers.PoseEstimate latestEstimate;
    if (!useMegaTag2 || DriverStation.isDisabled()) {
      latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue(name);
    } else {
      latestEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(name);
    }

    if (latestEstimate == null)
      return Optional.empty();

    if (latestEstimate.tagCount == 0)
      return Optional.empty();

    return Optional.of(
        new AprilTagResult(latestEstimate.pose,
            Utils.fpgaToCurrentTime(latestEstimate.timestampSeconds),
            latestEstimate.avgTagDist,
            latestEstimate.tagCount,
            latestEstimate.rawFiducials[0].ambiguity)); // Probably not the best but good enough for now
  }
}
