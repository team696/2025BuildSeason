// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Rotation;

import java.util.Arrays;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.HumanControls;

/**
 * A class contianing all the positions needed to move the superstructure into a
 * scoring position
 */
public class GameInfo {
    public static class CoralScoringPosition {
        /**
         * Creates a new CoralScoringPosition
         * 
         * @param height   The height of the elevator (zero represents the elevator's
         *                 lowest position), positive=up
         * @param armRot   The rotation of the arm
         * @param wristRot The rotation of the wrist
         */
        public CoralScoringPosition(double height, double armRot, double wristRot) {
            this.height = height;
            this.armRot = Rotation.of(armRot);
            this.wristRot = Rotation.of(wristRot);
        }

        public CoralScoringPosition() {

        }

        public double height;
        /* As measured from the vertical normal (arm facing up) */
        public Angle armRot;
        public Angle wristRot;
    }

    public static CoralScoringPosition L1, L2, L3, L4;

    public static class FieldSide {
        public Pose2d[] left, right, both;
    }

    public static FieldSide red, blue;

    public static FieldSide getFieldSide() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue ? blue : red;
    }

    public static Pose2d[] getScoringPoses() {
        return HumanControls.OperatorPanel2025.leftOrRight.getAsBoolean() ? getFieldSide().left : getFieldSide().right;
    }

    static {
        blue = new FieldSide();
        // TODO: determine the real scoring poses
        blue.left = new Pose2d[] {
                new Pose2d(3.539, 4.912, Rotation2d.fromDegrees(-60 + 180)),
                new Pose2d(3.179, 3.752, Rotation2d.fromDegrees(0 + 180)),
                new Pose2d(4.056, 2.689, Rotation2d.fromDegrees(60 + 180)),
                new Pose2d(5.450, 2.992, Rotation2d.fromDegrees(120 + 180)),
                new Pose2d(5.850, 4.376, Rotation2d.fromDegrees(180 + 180)),
                new Pose2d(4.924, 5.351, Rotation2d.fromDegrees(-120 + 180))
        };
        blue.right = new Pose2d[] {
                new Pose2d(4.046, 5.322, Rotation2d.fromDegrees(-60 + 180)),
                new Pose2d(3.188, 4.473, Rotation2d.fromDegrees(0 + 180)),
                new Pose2d(3.481, 3.050, Rotation2d.fromDegrees(60 + 180)),
                new Pose2d(4.836, 2.679, Rotation2d.fromDegrees(120 + 180)),
                new Pose2d(5.821, 3.762, Rotation2d.fromDegrees(180 + 180)),
                new Pose2d(5.489, 4.932, Rotation2d.fromDegrees(-120 + 180))
        };
        blue.both = Stream.concat(Arrays.stream(blue.left), Arrays.stream(blue.right))
                .toArray(size -> new Pose2d[size]);

        // The red poses are mirrored from the blue scoring poses
        red = new FieldSide();
        red.left = Arrays.stream(blue.left).map(pose -> PoseUtil.mirrorPoseX(pose)).toArray(size -> new Pose2d[size]);

        red.right = Arrays.stream(blue.right).map(pose -> PoseUtil.mirrorPoseX(pose)).toArray(size -> new Pose2d[size]);

        red.both = Stream.concat(Arrays.stream(red.left), Arrays.stream(red.right)).toArray(size -> new Pose2d[size]);

        // TODO: determine the real scoring heights
        L1 = new CoralScoringPosition(10., -4., -3.);
        L2 = new CoralScoringPosition(20., -4., -3.);
        L3 = new CoralScoringPosition(40., -4., -3.);
        L4 = new CoralScoringPosition(55., -4., -3.);
        /**
         * this puts the values on networktables so scoring positions can be quickly
         * changed
         * comment this out once the scoring positions are finalized
         */
        new TriggerNTDouble("testing/L1/height", L1.height, height -> L1.height = height);
        new TriggerNTDouble("testing/L2/height", L2.height, height -> L2.height = height);
        new TriggerNTDouble("testing/L3/height", L3.height, height -> L3.height = height);
        new TriggerNTDouble("testing/L4/height", L4.height, height -> L4.height = height);
        new TriggerNTDouble("testing/L1/armRot", L1.armRot.in(Rotation), rot -> L1.armRot = Rotation.of(rot));
        new TriggerNTDouble("testing/L2/armRot", L2.armRot.in(Rotation), rot -> L2.armRot = Rotation.of(rot));
        new TriggerNTDouble("testing/L3/armRot", L3.armRot.in(Rotation), rot -> L3.armRot = Rotation.of(rot));
        new TriggerNTDouble("testing/L4/armRot", L4.armRot.in(Rotation), rot -> L4.armRot = Rotation.of(rot));
        new TriggerNTDouble("testing/L1/wristRot", L1.wristRot.in(Rotation), rot -> L1.wristRot = Rotation.of(rot));
        new TriggerNTDouble("testing/L2/wristRot", L2.wristRot.in(Rotation), rot -> L2.wristRot = Rotation.of(rot));
        new TriggerNTDouble("testing/L3/wristRot", L3.wristRot.in(Rotation), rot -> L3.wristRot = Rotation.of(rot));
        new TriggerNTDouble("testing/L4/wristRot", L4.wristRot.in(Rotation), rot -> L4.wristRot = Rotation.of(rot));

    }

}
