// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import static edu.wpi.first.units.Units.Degree;

import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.HumanControls;

/** Add your docs here. */
public class GameInfo {
    public static class CoralScoringPosition{
        // TODO: go back to distance later
        //Distance height;
        double height;// will be rotations from default for now
        Angle rot;
    }
    public static CoralScoringPosition L1, L2, L3, L4;
    public static class FieldSide{
        public Pose2d[] left, right;
    }
    public static FieldSide red, blue;
    public static FieldSide getFieldSide(){
        // TODO: Cache the alliance station value somewhere when the match starts
        return DriverStation.getAlliance().orElse(Alliance.Blue)==Alliance.Blue?blue:red;
    }
    public static Pose2d[] getScoringPoses(){
        return HumanControls.DriverStation.leftOrRight.getAsBoolean()?getFieldSide().left:getFieldSide().right;
    }
    static{
        blue=new FieldSide();
        blue.left=new Pose2d[]{
            new Pose2d(3.539,4.912, Rotation2d.fromDegrees(-60 )),
            new Pose2d(3.179,3.752,Rotation2d.fromDegrees(0)),
            new Pose2d(4.056, 2.689, Rotation2d.fromDegrees(60)),
            new Pose2d(5.450,2.992,Rotation2d.fromDegrees(120)),
            new Pose2d(5.850,4.376,Rotation2d.fromDegrees(180)),
            new Pose2d(4.924,5.351,Rotation2d.fromDegrees(-120))
        };
        blue.right=new Pose2d[]{
            new Pose2d(4.046,5.322, Rotation2d.fromDegrees(-60 )),
            new Pose2d(3.188,4.473,Rotation2d.fromDegrees(0)),
            new Pose2d(3.481,3.050, Rotation2d.fromDegrees(60)),
            new Pose2d(4.836,2.679,Rotation2d.fromDegrees(120)),
            new Pose2d(5.821,3.762,Rotation2d.fromDegrees(180)),
            new Pose2d(5.489,4.932,Rotation2d.fromDegrees(-120))
        };
        /*red.left=new Pose2d[]{
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
        };
        red.right=new Pose2d[]{
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d(),
            new Pose2d()
        };*/
        L1=new CoralScoringPosition();
        L1.height=10;
        L1.rot=Degree.of(12.0);
        L2=new CoralScoringPosition();
        L2.height=15;
        L2.rot=Degree.of(12.0);
        L3=new CoralScoringPosition();
        L3.height=20;
        L3.rot=Degree.of(12.0);
        L4=new CoralScoringPosition();
        L4.height=25;
        L4.rot=Degree.of(12.0);
    } 
}
