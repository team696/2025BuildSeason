package frc.team696.lib.Swerve.Commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.team696.lib.Util;
import frc.team696.lib.Swerve.SwerveConstants;
import frc.team696.lib.Swerve.SwerveDriveSubsystem;

/**
 * Swerve command for driving around during teleop
 * 
 * <p> Config once and create multiple objects to handle different driving styles
 */
public class TeleopSwerve extends Command {
    protected static DoubleSupplier translation = ()->0;
    protected static DoubleSupplier strafe = ()->0;
    protected static DoubleSupplier rotation = ()->0;
    protected static double deadband = 1; // deadband for controller -> defaulted to 1 so you must config swerve
    protected static double rotationDeadband = 1;
    protected static SwerveDriveSubsystem swerveSubsystem = null;
    protected static double lastPeriodicSeconds = 0;
    private static PIDController pidController = new PIDController(0.0056, 0.00, 0); 
    static {
        pidController.enableContinuousInput(-180, 180);
        pidController.setTolerance(1);
    }
    private static BooleanSupplier lockRotation; // should lock rotation -> usually a button

    private boolean fieldRelative; // should do fieldRelative controol
    private boolean openLoop; // should do openLoop control
    private Supplier<Rotation2d> rotationGoal; // Rotation to lock to once lockRotation has been activated
    private DoubleSupplier multiplier = ()->1; // Multiplier to outputted Speed

    public static void config(SwerveDriveSubsystem s, DoubleSupplier x, DoubleSupplier y, DoubleSupplier r, BooleanSupplier rotationLock, double deadBand) {
        strafe = x;
        translation = y;
        rotation = r;

        lockRotation = rotationLock;

        deadband = deadBand;
        rotationDeadband = Math.sqrt(2 * Math.pow(deadband, 2));
        swerveSubsystem = s;
    }

    public static TeleopSwerve New(){
        return new TeleopSwerve();
    }
    
    private TeleopSwerve() {
        this.fieldRelative = true;
        this.openLoop = true;

        this.rotationGoal = ()->null;

        this.multiplier = ()->1;

        addRequirements(swerveSubsystem);
    }

    public TeleopSwerve withMultiplier(DoubleSupplier multiplier) {
        this.multiplier = multiplier;
        return this;
    }

    public TeleopSwerve withMultiplier(double multiplier) {
        this.multiplier = ()->multiplier;
        return this;
    }

    public TeleopSwerve withRotationGoal(Supplier<Rotation2d> goal) {
        this.rotationGoal = goal;
        return this;
    }

    public TeleopSwerve withRotationGoal(Rotation2d goal) {
        this.rotationGoal = ()->goal;
        return this;
    }

    public TeleopSwerve withRobotRelative(boolean robotRelative) {
        this.fieldRelative = !robotRelative;
        return this;
    }

    public TeleopSwerve withOpenLoop(boolean openLoop) {
        this.openLoop = openLoop;
        return this;
    }

    @Override
    public void execute() {
        double yAxis = translation.getAsDouble();
        double xAxis = strafe.getAsDouble();
        double rAxis = rotation.getAsDouble();

        Rotation2d theta=Rotation2d.fromDegrees(0);
        if(xAxis!=0||yAxis!=0)
             theta = new Rotation2d(yAxis, xAxis);
        double magnitude = Math.min(Math.sqrt((xAxis * xAxis) + (yAxis * yAxis)), 1);
        if (magnitude < deadband) magnitude = 0;
        Rotation2d goalRotation = rotationGoal.get();
        if (lockRotation != null && lockRotation.getAsBoolean() && goalRotation != null) { // Rotation Lock To Angle 
            double pid = pidController.calculate(swerveSubsystem.getPose().getRotation().getDegrees(), goalRotation.getDegrees());
            rAxis = Math.abs(pidController.getError()) > 1 ? Math.abs(Math.pow(pid, 2)) * 1.1 * Math.signum(pid) + pid * 2.2 : 0;
        } else {
            rAxis = (Math.abs(rAxis) > rotationDeadband) ? Util.map(rAxis * rAxis, rotationDeadband, 1, 0, 1) * Math.signum(rAxis) : 0;
        }

        double outputPercent = Math.min(multiplier.getAsDouble(),1);

        double desiredRotation = rAxis * SwerveConstants.MAX_ANGULAR_VELOCITY.in(RadiansPerSecond);
        Translation2d desiredTranslation = new Translation2d(Math.pow(magnitude, 2), theta).times(SwerveConstants.MAX_VELOCITY.in(MetersPerSecond)).times(outputPercent);

        /* Untested Portion Begin 
        double deltaSeconds = Timer.getFPGATimestamp() - lastPeriodicSeconds;

        // Acceleration Limiting Start 
        Translation2d curVel = swerveSubsystem.getState().velocityXY();
        Translation2d desiredAcceleration = desiredTranslation.minus(curVel).div(deltaSeconds);
        double accelerationLimit = SwerveConstants.MAX_ACCELERATION.in(MetersPerSecondPerSecond) * (1 - curVel.getNorm() / SwerveConstants.MAX_VELOCITY.in(MetersPerSecond));

        desiredAcceleration = desiredAcceleration.div(desiredAcceleration.getNorm()).times(accelerationLimit);

        double accelerationXYScaling = Math.max(Math.max(1 , Math.abs(desiredAcceleration.getX()) / SwerveConstants.MAX_ACCELERATION_X.in(MetersPerSecondPerSecond) ), Math.abs(desiredAcceleration.getY()) / SwerveConstants.MAX_ACCELERATION_Y.in(MetersPerSecondPerSecond));
        desiredAcceleration = desiredAcceleration.div(desiredAcceleration.getNorm()).times(accelerationXYScaling);

        double maxSkidAcceleration = Math.max(1, desiredAcceleration.getNorm() / SwerveConstants.MAX_ACCELERATION_SKID.in(MetersPerSecondPerSecond));
        desiredAcceleration = desiredAcceleration.div(desiredAcceleration.getNorm()).times(maxSkidAcceleration);

        // Acceleration Limiting End 
        
        // Jerk Limiting Start 
        // Might do Questionable Things because we are using our currentAcceleration which may not be accurate (Why it is in the untested Portion) 

        Translation2d curAccel = swerveSubsystem.getState().accelerationXY();
        Translation2d desiredJerk = desiredAcceleration.minus(curAccel).div(deltaSeconds);
        double jerkLimit = SwerveConstants.MAX_JERK.in(MetersPerSecondPerSecond.per(Seconds));
        
        double jerkScaling = Math.min(jerkLimit, desiredJerk.getNorm());

        Translation2d scaledJerk = desiredJerk.div(desiredJerk.getNorm()).times(jerkScaling);

        desiredAcceleration = curAccel.plus(scaledJerk.times(deltaSeconds));
        
        // Jerk Limiting End 

        desiredTranslation = curVel.plus(desiredAcceleration.times(deltaSeconds));

        lastPeriodicSeconds = Timer.getFPGATimestamp();
        /* Untested Portion End */

        swerveSubsystem.Drive(desiredTranslation, desiredRotation, fieldRelative, openLoop);

    }
}