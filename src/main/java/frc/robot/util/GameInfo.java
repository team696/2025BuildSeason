// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;
import java.util.stream.Stream;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
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

        public double height;
        // Rotations of the Motor, Im too lazy to change off angle now
        public Angle armRot; 
        public Angle wristRot;
    }

    public static CoralScoringPosition L1, L2, L3, L4, Net, Intake, ground, ClimbUp, ClimbDown;


    /* Looking at the Index Dead On */
    public enum Side {
        Right,
        Left
    }

    /*  Starts from 3 O'clock on field render */
    public enum Index {
        One, 
        Two, 
        Three,
        Four,
        Five, 
        Six
    }

    public final static Distance fieldLengthMeters = Feet.of(57.53);

    public static Translation2d mirrorTranslation(Translation2d starting) {
        return new Translation2d(fieldLengthMeters.in(Meters) - starting.getY(), starting.getY());
    } 

    public final static Translation2d blueReef = new Translation2d(4.5,4.);

    public final static Map<Index, Map<Side, Translation2d>> ScoringPosesBlue;
    
    static {
        ScoringPosesBlue = Map.of(
            Index.One, Map.of(
                Side.Right, new Translation2d(4.046, 5.322),
                Side.Left, new Translation2d(3.539, 4.912)
            ),
            
            Index.Two, Map.of(
                Side.Right, new Translation2d(3.188, 4.473),
                Side.Left, new Translation2d(3.179, 3.752)
            ),
            
            Index.Three, Map.of(
                Side.Right, new Translation2d(3.481, 3.050),
                Side.Left, new Translation2d(4.056, 2.689)
            ),
            
            Index.Four, Map.of(
                Side.Right, new Translation2d(4.836, 2.679),
                Side.Left, new Translation2d(5.450, 2.992)
            ),
            
            Index.Five, Map.of(
                Side.Right, new Translation2d(5.821, 3.762),
                Side.Left, new Translation2d(5.850, 4.376)
            ),
            
            Index.Six, Map.of(
                Side.Right, new Translation2d(5.489, 4.932),
                Side.Left, new Translation2d(4.924, 5.351)
            )
        );



        L1 = new CoralScoringPosition(0., 1.75, 0.45);
        L2 = new CoralScoringPosition(8., 1.75, 0.45);
        L3 = new CoralScoringPosition(30., 1.75, .45);
        L4 = new CoralScoringPosition(67., 0.7, .65);
        Net = new CoralScoringPosition(67., 0., 2.5);
        ClimbUp=new CoralScoringPosition(27, -8., 0);
        ClimbDown=new CoralScoringPosition(2, -8., 0);
        Intake = new CoralScoringPosition(0, -1.75, 3.6);
        ground=new CoralScoringPosition(5., 8.2, 4.3);
    }

}
