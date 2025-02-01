package frc.robot;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Seconds;
import static edu.wpi.first.units.Units.Volts;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.team696.lib.Dashboards.ShuffleDashboard;
import frc.team696.lib.Logging.BackupLogger;
import frc.team696.lib.Logging.PLog;
import frc.team696.lib.Swerve.SwerveConfigs;
import frc.team696.lib.Swerve.SwerveConstants;

/**
 * Houses all methods related to the Autonomous period and self driving during teleop
 */
@SuppressWarnings("resource")
public class Auto {
    /**
     * Wrapper For Pathplanners NamedCommand System
     */
    public static class NamedCommand {
        public String name;
        public Command command;

        /**
         * @param name to be used inside of the pathplanner GUI
         * @param command to be run when called inside pathplanner
         */
        public NamedCommand(String name, Command command) {
            this.name = name;
            this.command = command;
        }

        public void register() {
            NamedCommands.registerCommand(name, command);
        }
    }

    public static Auto _instance;
    private CommandSwerveDrivetrain _swerve;

    private SendableChooser<Command> _autoChooser;

    public SysIdRoutine _driveSysIdRoutine; 
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds();

    private Auto (CommandSwerveDrivetrain swerve, boolean shouldUseGUIValues, NamedCommand... commandsToRegister) {
        _swerve = swerve;

        RobotConfig config = new RobotConfig(
            SwerveConstants.MASS, 
            SwerveConstants.MOMENT_OF_INERTIA, 
            new ModuleConfig(
                SwerveConstants.WHEEL_RADIUS, 
                SwerveConstants.MAX_VELOCITY, 
                SwerveConstants.WHEEL_COEFFICIENT_OF_FRICTION, 
                DCMotor.getKrakenX60(1), 
                Amps.of(SwerveConfigs.drive.CurrentLimits.StatorCurrentLimit), 
                1), 
            SwerveConstants.modPositions);
        if (shouldUseGUIValues) { 
            try{
                config = RobotConfig.fromGUISettings();
            } catch (Exception e) {
                PLog.fatalException("Auto", "Failed To Get RobotConfig", e); 
                new Alert("Failed to fetch robotConfig GUI Settings", AlertType.kWarning).set(true);
            }
        }

        AutoBuilder.configure(
            ()->_swerve.getState().Pose, 
            _swerve::resetPose, 
            ()->_swerve.getState().Speeds,
            (speeds, feedforwards) -> {m_pathApplyRobotSpeeds.withSpeeds(speeds)
            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons()); System.out.println("medina more like megdina");}, 
            new PPHolonomicDriveController(  
                    new PIDConstants(10.0, 0.0, 0.0), 
                    new PIDConstants(7.0, 0.0, 0.0)),
            config,
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            _swerve
        );

        for (NamedCommand namedCommand : commandsToRegister) {
            namedCommand.register();
        }

        PathPlannerLogging.setLogTargetPoseCallback((pose) -> {
            BackupLogger.addToQueue("696/Auto/Desired", pose);
        });

        PathPlannerLogging.setLogActivePathCallback((poses) -> {
            ShuffleDashboard.field.getObject("Path").setPoses(poses);
        });

        _autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData(_autoChooser);   
        //ShuffleDashboard.addAutoChooser(_autoChooser);

        _autoChooser.onChange((command)-> {
            visualize();
        });
    }
    /**
     * Initializes all things related to auto (ex. pathplanner)
     * 
     * @param swerve The swerve subsystem
     * @param commandsToRegister each command to register for path planner
     */
    public static void Initialize(CommandSwerveDrivetrain swerve, NamedCommand... commandsToRegister){
        if (_instance != null) throw new RuntimeException ("Don't Initialize Twice!");
        
        _instance = new Auto(swerve, false, commandsToRegister);
    }

    /**
     * Initializes all things related to auto (ex. pathplanner)
     * 
     * @param swerve The swerve subsystem
     * @param shouldUseGUIValues Should Pathplanner fetch Robot Config from GUI?
     * @param commandsToRegister each command to register for path planner
     */
    public static void Initialize(CommandSwerveDrivetrain swerve, boolean shouldUseGUIValues, NamedCommand... commandsToRegister){
        if (_instance != null) throw new RuntimeException ("Don't Initialize Twice!");
        
        _instance = new Auto(swerve, shouldUseGUIValues, commandsToRegister);
    }

    /**
     * @return The instance Of the Auto Class
     */
    public static Auto get(){
        if (_instance == null) throw new RuntimeException ("Please Initialize First!");

        return _instance;
    }

    /**
     * @return Selected Command From SendableChooser
     */
    public Command Selected() {
        return _autoChooser.getSelected();
    }

    /**
     * @return Selected Command From SendableChooser Which Will End After 15 Seconds.
     *              Used for testing on home field
     */
    public Command SelectedEndAt15() {
        return Selected().raceWith(new WaitCommand(15.0));
    }

    public static Command PathFind(Pose2d end) {
        return AutoBuilder.pathfindToPose(end, new PathConstraints(1, 1, Math.PI,Math.PI));
    }

    public void visualize(String name) {
        List<PathPlannerPath> paths;
        List<Pose2d> pathPoses = new ArrayList<Pose2d>();
        try {
            paths = PathPlannerAuto.getPathGroupFromAutoFile(name);
        } catch (Exception e) {
            PLog.fatalException("Auto", "Failed To Find Path", e);
            return;
        }
        for (int i = 0; i < paths.size(); i++) {
            PathPlannerPath path = paths.get(i);
            if (DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
                path = path.flipPath();
            pathPoses.addAll(path.getPathPoses());
        }
        ShuffleDashboard.field.getObject("traj").setPoses(pathPoses);
    }

    public void visualize() {
        visualize(_autoChooser.getSelected().getName());
    }


}