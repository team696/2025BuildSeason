// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.team696.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** 
 * Add Any Necessary Field Elements Below
 * 
 * To Be Updated Each Year
 */
public class Field {
    public static abstract class FieldSide {
        public abstract Pose2d FieldElement();
    }

    public static final class RED extends FieldSide {
        @Override
        public final Pose2d FieldElement() {
            return new Pose2d();
        }
    }
    public static final class BLUE extends FieldSide {
        @Override
        public final Pose2d FieldElement() {
            return new Pose2d();
        }    
    }

    public static final RED redInstance = new RED();
    public static final BLUE blueInstance = new BLUE();

    public static final FieldSide getSide() {
        if (Util.getAlliance() == Alliance.Blue) {
            return blueInstance;
        } 
        return redInstance;
    }

    public static final FieldSide getOppositeSide() {
        if (Util.getAlliance() == Alliance.Blue) {
            return redInstance;
        } 
        return blueInstance;
    }

}
