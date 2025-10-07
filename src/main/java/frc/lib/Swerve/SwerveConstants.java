package frc.team696.lib.Swerve;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilogram;
import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.Minute;
import static edu.wpi.first.units.Units.NewtonMeters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotation;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.AngleUnit;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.LinearAccelerationUnit;
import edu.wpi.first.units.LinearVelocityUnit;
import edu.wpi.first.units.MassUnit;
import edu.wpi.first.units.MultUnit;
import edu.wpi.first.units.TimeUnit;
import edu.wpi.first.units.VoltageUnit;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.units.measure.Mult;
import edu.wpi.first.units.measure.Per;
import edu.wpi.first.units.measure.Torque;
import edu.wpi.first.units.measure.Voltage;

/**
 * Constants of the Swerve Drive
 * 
 * <p> Should Be Updated For Each Robot Accordingly
 */
public class SwerveConstants {
		public static final String canBus = "cv";

		public static final int expectedModuleCount = 4;

		public static final Voltage drivekS = Volts.of(0.667); 
		public static final Per<VoltageUnit, LinearVelocityUnit> drivekV = Volts.of(2.44).div(MetersPerSecond.of(1));
		public static final Per<VoltageUnit, LinearAccelerationUnit> drivekA = Volts.of(0.27).div(MetersPerSecondPerSecond.of(1));

		public static final double DRIVE_GEAR_RATIO = (5.36 / 1.0);
		public static final double ANGLE_GEAR_RATIO = (150.0 / 7.0 / 1.0); 
	
		public static final Mass MASS = Pounds.of(140);

		// Distance From The Center To The Wheel, From The Distance Between Two Wheels divided By 2
		public static final Distance WHEELBASE_X = Inches.of(18.5).div(2);
		public static final Distance WHEELBASE_Y = Inches.of(18.5).div(2);
		public static final Distance DRIVEBASE_RADIUS = Inches.of(Math.sqrt(Math.pow(WHEELBASE_X.in(Inches), 2) + Math.pow(WHEELBASE_Y.in(Inches), 2)));
		public static final Distance WHEEL_DIAMETER = Inches.of(3.5);
		public static final Distance WHEEL_RADIUS = WHEEL_DIAMETER.div(2);
		public static final Distance WHEEL_CIRCUM = WHEEL_DIAMETER.times(Math.PI);
		public static final double WHEEL_COEFFICIENT_OF_FRICTION = 1.2;
	
		public static final Per<DistanceUnit, AngleUnit> DISTANCE_PER_ROTATION = WHEEL_CIRCUM.div(DRIVE_GEAR_RATIO).div(Rotations.of(1));

		// Use Reca.lc to find Stall Torque At Motor Current Limit
		// Values For Kraken X60
		public static final AngularVelocity MAX_FREESPIN_VELOCITY = Rotations.of(6000).per(Minute);
																			// Current Limit / Max Current / Torque At Max Current
		public static final Torque MAX_MOTOR_STALL_TORQUE = NewtonMeters.of(SwerveConfigs.drive.CurrentLimits.StatorCurrentLimit / 366 * 7.09); 
		public static final Force MAX_WHEEL_FORCE = MAX_MOTOR_STALL_TORQUE.times(DRIVE_GEAR_RATIO).div(WHEEL_RADIUS).times(expectedModuleCount);

		public static final LinearVelocity THEORETICAL_MAX_SPEED = MetersPerSecond.of(MAX_FREESPIN_VELOCITY.in(RotationsPerSecond) * DISTANCE_PER_ROTATION.in(Meters.per(Rotation))); 
		public static final LinearAcceleration THEORETICAL_MAX_ACCELERATION = (MAX_WHEEL_FORCE.div(MASS));  
		public static final LinearVelocity MAX_VELOCITY = THEORETICAL_MAX_SPEED.times(0.85); 
		public static final LinearAcceleration MAX_ACCELERATION = THEORETICAL_MAX_ACCELERATION.times(0.7); 
		public static final AngularVelocity THEORETICAL_MAX_ANGULAR_VELOCITY = RotationsPerSecond.of(MAX_VELOCITY.in(MetersPerSecond) / (DRIVEBASE_RADIUS.in(Meters) * 2 * Math.PI));
		public static final AngularVelocity MAX_ANGULAR_VELOCITY = THEORETICAL_MAX_ANGULAR_VELOCITY.times(0.95); 
		
		/* Credit To 1690s Software Session Part 1
		 * Forward limit -> Max Acceleration Magnitude -> MaxAcc * (1 - curVel / maxVel)-> see TeleopSwerve.java for usage
		 * Tilt limit -> Limits Magnitude of Accelx and Accely -> MaxFrontAccel & MaxSideAcc
		 * Skid Limit -> Limits Max Acceleration Of the robot to prevent skidding
		 */
		public static final LinearAcceleration MAX_ACCELERATION_X =    MetersPerSecondPerSecond.of(10000000);
		public static final LinearAcceleration MAX_ACCELERATION_Y =    MetersPerSecondPerSecond.of(10000000);
		public static final LinearAcceleration MAX_ACCELERATION_SKID = MetersPerSecondPerSecond.of(10000000); 
		
		/**
		 * 	Jerk Control - My Own
		 */
		public static final Per<LinearAccelerationUnit, TimeUnit> MAX_JERK = MetersPerSecondPerSecond.per(Seconds).ofNative(100000000);

		//ASSUMES UNIFORM DISTRIBUTION, SHOULD BE CALCULATED EXPERIMENTALLY
		public static final Mult<MultUnit<MassUnit, DistanceUnit>, DistanceUnit> THEORETICAL_MOMENT_OF_INERTIA = MASS.times(DRIVEBASE_RADIUS).times(DRIVEBASE_RADIUS).times(1/12);

		/** 
		 * Credit (https://pathplanner.dev/robot-config.html#module-config-options)
		 * 
		 * Calculate Kangular and Klinear through SysID
		 * 
		 * Mass * WheelBase_X * Kangular * Klinear
		*/
		public static final MomentOfInertia MOMENT_OF_INERTIA = MomentOfInertia.ofBaseUnits(THEORETICAL_MOMENT_OF_INERTIA.in(MultUnit.combine(MultUnit.combine(Kilogram, Meters), Meters)), KilogramSquareMeters);

		public static final Translation2d[] modPositions = {
			new Translation2d(WHEELBASE_X, WHEELBASE_Y), // FL
			new Translation2d(WHEELBASE_X, WHEELBASE_Y.times(-1)), // FR
			new Translation2d(WHEELBASE_X.times(-1), WHEELBASE_Y), // BL
			new Translation2d(WHEELBASE_X.times(-1), WHEELBASE_Y.times(-1)) // BR  
		};

}
