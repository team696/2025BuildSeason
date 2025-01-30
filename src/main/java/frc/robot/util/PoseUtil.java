// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;

/** Add your docs here. */
public class PoseUtil {

        /**
     * Finds the closest Pose2d to a reference Pose2d from a list of Pose2ds.
     * "Closest" is determined by the Euclidean distance between the translation components (x, y) of the poses.
     * Rotation is not considered in the distance calculation.
     *
     * @param poseList     The list of Pose2ds to search within. Must not be null or empty.
     * @param referencePose The Pose2d to find the closest Pose2d to. Must not be null.
     * @return The Pose2d from the list that is closest to the referencePose, or null if the list is null or empty.
     * @throws IllegalArgumentException if poseList or referencePose is null.
     * @throws IllegalStateException if poseList is empty.
     */
    public static Pose2d findClosestPose(List<Pose2d> poseList, Pose2d referencePose) {
        if (poseList == null) {
            throw new IllegalArgumentException("poseList cannot be null");
        }
        if (referencePose == null) {
            throw new IllegalArgumentException("referencePose cannot be null");
        }
        if (poseList.isEmpty()) {
            return null; // Or throw an exception: throw new IllegalStateException("poseList cannot be empty");
        }

        Pose2d closestPose = null;
        double minDistance = Double.MAX_VALUE; // Initialize with a very large value

        for (Pose2d pose : poseList) {
            // Calculate the squared Euclidean distance between the translations
            Translation2d poseTranslation = pose.getTranslation();
            Translation2d referenceTranslation = referencePose.getTranslation();
            double distance = referenceTranslation.getDistance(poseTranslation);

            if (distance < minDistance) {
                minDistance = distance;
                closestPose = pose;
            }
        }

        return closestPose;
    }
}
