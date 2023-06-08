package frc.robot.commands.Routines;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.RobotContainer;
import frc.robot.commands.Routines.StateTypes.PositionState;
import frc.robot.subsystems.Swerve.SwerveDrive;
import frc.robot.util.OdometryMath2023;

public class MoveBotTo extends CommandBase {

    private Pose2d setpoint;
    private Pose2d measurement;
    private SwerveDrive swerve;
    public static boolean running = false;
    public static double runTime = 0;
    public static double runningSpeed = 0;
    private boolean switchToTape = false;
    private static ProfiledPIDController xController, yController, thetaController;
    private Timer timer;
    private static RoutineConstants.POSITION_TYPE lastPositionType;
    private RoutineConstants.POSITION_TYPE setType;
    private boolean isJank;
    private RoutineConstants.POSITION_TYPE type;
    public static boolean goodToRelease;

    public MoveBotTo(RoutineConstants.POSITION_TYPE type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        isJank = false;
        this.type = type;
        this.setType = type;
        running = false;
    }

    public MoveBotTo(Pose2d type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        isJank = true;
        this.setpoint = type;
        running = false;
    }

    @Override
    public void initialize() {
        goodToRelease = false;
        running = true;
        if (!isJank) {
            this.setpoint = PositionState.getPositionPose(type);
        }
        initControllers();
        // runTime = 0;
        timer.reset();
        timer.start();
        OdometryMath2023.reseedOdometry();
        xController.setGoal(setpoint.getX());
        yController.setGoal(setpoint.getY());
        thetaController.setGoal(setpoint.getRotation().getRadians());
        switchToTape = false;
    }

    @Override
    public void execute() {
        runTime = timer.get();
        measurement = swerve.getPose();
        double xCalc = xController.calculate(measurement.getX());
        double yCalc = yController.calculate(measurement.getY());
        double thetaCalc;
        if ((!switchToTape) || (!OdometryMath2023.facingForward(5))) {
            if (
                Math.hypot(xCalc, yCalc) < RoutineConstants.RESEED_SPEED_THRESHOLD &&
                Math.abs(measurement.getRotation().minus(setpoint.getRotation()).getRadians()) < Math.PI / 6 &&
                runTime > 0.75 && 
                OdometryMath2023.tagRotation() != null
            ) {
                thetaCalc = thetaController.calculate(OdometryMath2023.tagRotation().getRadians());
            } else {
                thetaCalc = thetaController.calculate(measurement.getRotation().getRadians());
            }    
        } else {
            thetaCalc = RobotContainer.getLimelightTape().getAutoAlignCalc();
            if (
                Math.hypot(xCalc, yCalc) < RoutineConstants.RESEED_SPEED_THRESHOLD &&
                Math.abs(measurement.getRotation().minus(setpoint.getRotation()).getRadians()) < Math.PI / 6 &&
                runTime > 0.75 && 
                OdometryMath2023.tagRotation() != null
            ) {
                thetaController.reset(OdometryMath2023.tagRotation().getRadians());
            } else {
                thetaController.reset(measurement.getRotation().getRadians());
            }
        }

        if (
            Math.abs(xController.getPositionError()) < (RoutineConstants.TRANSLATION_TOLERANCE_METERS + 0.02) && 
            Math.abs(yController.getPositionError()) < (RoutineConstants.TRANSLATION_TOLERANCE_METERS + 0.02) &&
            Math.abs(thetaController.getPositionError()) < (RoutineConstants.ROTATION_TOLERANCE_RAD + Math.toRadians(3))
        ) {
            switchToTape = true;
        }

        if(
            Math.hypot(xCalc, yCalc) < RoutineConstants.RESEED_SPEED_THRESHOLD &&
            runTime > 0.75
        ){
            OdometryMath2023.reseedOdometry();
        }

        SwerveModuleState[] states;
        if (
            Math.abs(measurement.getRotation().minus(setpoint.getRotation()).getRadians()) < RoutineConstants.ROT_THRESH_RAD
        ) {
            states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    xCalc, 
                    yCalc, 
                    thetaCalc, 
                    swerve.getRotation2d()
                )
            );
            runningSpeed = Math.hypot(xCalc, yCalc);
        } else {
            states = SwerveConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    0, 
                    0, 
                    thetaCalc, 
                    swerve.getRotation2d()
                )
            );
            runningSpeed = 0;
            xController.reset(measurement.getX());
            yController.reset(measurement.getY());
        }

        swerve.setModules(states);
    }

    @Override
    public boolean isFinished() {

        runTime = timer.get();
        if(RobotContainer.getController().getHID().getPOV() != -1){
            return true;
        }

        if (
            (Math.abs(xController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && xController.getSetpoint().equals(xController.getGoal())) && 
            (Math.abs(yController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && yController.getSetpoint().equals(yController.getGoal())) && 
            ((
                Math.abs(thetaController.getPositionError()) < RoutineConstants.ROTATION_TOLERANCE_RAD && 
                thetaController.getSetpoint().equals(thetaController.getGoal()) &&
                !switchToTape
            ) || (
                switchToTape &&
                Math.abs(RobotContainer.getLimelightTape().getTX()) < 1
            ))
        ) {

            lastPositionType = this.setType;
            
            if ((runTime < RoutineConstants.MOVE_BOT_TO_REPEAT_THRESHOLD_SEC)) {
                goodToRelease = true;
                return true;
            } else{
                end(false);
                initialize();
                return false;

            }
            
        } else if (RobotContainer.getController().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D) || RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_O)) {
            return true;
        } else {
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        if (OdometryMath2023.tagRotation() != null) {
            swerve.setHeadingRaw(OdometryMath2023.tagRotation().getRadians());
        }
        OdometryMath2023.reseedOdometry();
        swerve.stopMods();
        runTime = timer.get();
        timer.stop();
        timer.reset();
        running = false;
        runningSpeed = 0;
        switchToTape = false;
    }

    private void initControllers() {
        xController = new ProfiledPIDController(
            RoutineConstants.TRANSLATION_P, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S/1.25, 
                RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S/1.25
            )
        );
        xController.setTolerance(RoutineConstants.TRANSLATION_TOLERANCE_METERS);
        xController.reset(swerve.getPose().getX());

        yController = new ProfiledPIDController(
            RoutineConstants.TRANSLATION_P, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S,
                RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
            )
        );
        yController.setTolerance(RoutineConstants.TRANSLATION_TOLERANCE_METERS);
        yController.reset(swerve.getPose().getY());

        thetaController = new ProfiledPIDController(
            RoutineConstants.ROTATION_P, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_ROTATION_SPEED_RAD_S, 
                RoutineConstants.ROUTINE_MAX_ROTATION_ACCEL_RAD_S_S
            )
        );
        thetaController.setTolerance(RoutineConstants.ROTATION_TOLERANCE_RAD);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        thetaController.reset(swerve.getPose().getRotation().getRadians());
    }

    public static RoutineConstants.POSITION_TYPE getLastPositionType() {
        return lastPositionType;
    }
}