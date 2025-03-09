// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Rotation;

import java.util.Map;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

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

    public static CoralScoringPosition Net, ground, Processor, ClimbUp, ClimbDown, L2Algae, L3Algae;


    /* Looking at the Index Dead On */
    public enum ReefSide {
        Right,
        Left
    }

    /*  Labels For Blue Side Of Field, Relative To Center Of Hex */
    public enum Index {
        One, // -X 
        Two,  // -X, +Y
        Three, // +X, +Y
        Four, // +X
        Five, // +X, -Y
        Six // -X, -Y
    }

    public final static Distance fieldLengthMeters = Feet.of(57.53);
    public final static Distance fieldWidthMeters = Feet.of(26.75);

    public static Translation2d mirrorTranslation(Translation2d starting) {
        return new Translation2d(fieldLengthMeters.in(Meters) - starting.getY(), starting.getY());
    } 

    public final static Translation2d blueReef = new Translation2d(4.5,4.);

    public final static Map<Index, Map<ReefSide, Translation2d>> ScoringPosesBlue;
    
    public enum Position {
        L1,
        L2, 
        L3,
        L4,
        Intake
    }

    public enum RobotSide {
        Front,
        Back
    }

    public final static Map<Position, Map<RobotSide, CoralScoringPosition>> RobotState;

    static {
        ScoringPosesBlue = Map.of(
            Index.One, Map.of(
                ReefSide.Right, new Translation2d(4.046, 5.322),
                ReefSide.Left, new Translation2d(3.539, 4.912)
            ),
            
            Index.Two, Map.of(
                ReefSide.Right, new Translation2d(3.188, 4.473),
                ReefSide.Left, new Translation2d(3.179, 3.752)
            ),
            
            Index.Three, Map.of(
                ReefSide.Right, new Translation2d(3.481, 3.050),
                ReefSide.Left, new Translation2d(4.056, 2.689)
            ),
            
            Index.Four, Map.of(
                ReefSide.Right, new Translation2d(4.836, 2.679),
                ReefSide.Left, new Translation2d(5.450, 2.992)
            ),
            
            Index.Five, Map.of(
                ReefSide.Right, new Translation2d(5.821, 3.762),
                ReefSide.Left, new Translation2d(5.850, 4.376)
            ),
            
            Index.Six, Map.of(
                ReefSide.Right, new Translation2d(5.489, 4.932),
                ReefSide.Left, new Translation2d(4.924, 5.351)
            )
        );

        RobotState = Map.of(
            Position.L1, Map.of(
                RobotSide.Front, new CoralScoringPosition(0., 1.75, 1.1),
                RobotSide.Back, new CoralScoringPosition(0, -1., -8.)
            ),
            Position.L2, Map.of(
                RobotSide.Front, new CoralScoringPosition(14., 1.75, 1.56),
                RobotSide.Back, new CoralScoringPosition(7., -1., -7.6)
            ),
            Position.L3, Map.of(
                RobotSide.Front, new CoralScoringPosition(33., 1.75, 1.1),
                RobotSide.Back, new CoralScoringPosition(25., -1., -7.9)
            ),
            Position.L4, Map.of(
                RobotSide.Front, new CoralScoringPosition(67., 0.7, 1.6),
                RobotSide.Back, new CoralScoringPosition(62, -1.1, -8.3)
            ),
            Position.Intake, Map.of(
                RobotSide.Front, new CoralScoringPosition(6., 1., -0.9),
                RobotSide.Back, new CoralScoringPosition(0, 0, 0)
            )
        );
        L2Algae = new CoralScoringPosition (18., -3., 1.);
        L3Algae = new CoralScoringPosition (44., -3., 1.);
        Net = new CoralScoringPosition(67., 0., 5.8);
        ClimbUp=new CoralScoringPosition(27, 0, 0);
        ClimbDown=new CoralScoringPosition(2, 0, 0);
        ground=new CoralScoringPosition(5., -6.3, -.5);
        Processor = new CoralScoringPosition(0, -5., 1.);
    }

}
