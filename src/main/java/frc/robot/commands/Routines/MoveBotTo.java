package frc.robot.commands.Routines;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.JoystickConstants;
import frc.robot.Constants.RoutineConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.RoutineConstants.POSITION_TYPE;
import frc.robot.RobotContainer;
import frc.robot.commands.Routines.StateTypes.PositionState;
import frc.robot.subsystems.Swerve.SwerveDrive;

public class MoveBotTo extends CommandBase {

    private Pose2d setpoint;
    private Pose2d measurement;
    private SwerveDrive swerve;
    public static boolean isRunning = false;
    public static double runningSpeed = 0;
    private static ProfiledPIDController xController, yController, thetaController;
    private Timer timer;
    private static RoutineConstants.POSITION_TYPE lastPositionType;
    private RoutineConstants.POSITION_TYPE setType;
    private boolean isJank;
    private RoutineConstants.POSITION_TYPE type;

    public MoveBotTo(RoutineConstants.POSITION_TYPE type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        isJank = false;
        this.type = type;
        // initControllers();
        this.setType = type;
        isRunning = false;
    }

    public MoveBotTo(Pose2d type) {
        swerve = RobotContainer.getSwerve();
        addRequirements(swerve);
        timer = new Timer();
        isJank = true;
        this.setpoint = type;
        isRunning = false;
    }

    @Override
    public void initialize() {
        isRunning = true;
        if (!isJank) {
            this.setpoint = PositionState.getPositionPose(type);
        }
        initControllers();
        timer.reset();
        timer.start();
        xController.setGoal(setpoint.getX());
        yController.setGoal(setpoint.getY());
        thetaController.setGoal(setpoint.getRotation().getRadians());
    }

    @Override
    public void execute() {
        measurement = swerve.getPose();
        double xCalc = xController.calculate(measurement.getX());
        double yCalc = yController.calculate(measurement.getY());
        double thetaCalc = thetaController.calculate(measurement.getRotation().getRadians());

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
        // Try this VVV Cancel command if driver joystick moved
        if(
            Math.abs(RobotContainer.getJoy().getHID().getX()) > JoystickConstants.CONTROLLER_DEADBAND ||
            Math.abs(RobotContainer.getJoy().getHID().getY()) > JoystickConstants.CONTROLLER_DEADBAND ||
            Math.abs(RobotContainer.getJoy().getHID().getX()) > JoystickConstants.CONTROLLER_DEADBAND
        ){
            return true;
        }
        // Also Try this VVV Cancel command if driver pov or operator pov pressed
        if(
            RobotContainer.getJoy().getHID().getPOV() != 0 ||
            RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.OPERATOR_STRAFE_FWD) ||
            RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.OPERATOR_STRAFE_BACK) ||
            RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.OPERATOR_STRAFE_LEFT) ||
            RobotContainer.getOperatorJoy1().getHID().getRawButton(JoystickConstants.OPERATOR_STRAFE_RIGHT)
        ){
            return true;
        }


        if (
            (Math.abs(xController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && xController.getSetpoint().equals(xController.getGoal())) && 
            (Math.abs(yController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && xController.getSetpoint().equals(xController.getGoal())) && 
            (Math.abs(thetaController.getPositionError()) < RoutineConstants.TRANSLATION_TOLERANCE_METERS && xController.getSetpoint().equals(xController.getGoal()))
        ) {
            lastPositionType = this.setType;
            return true;
        } else if (RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_D) || RobotContainer.getJoy().getHID().getRawButton(JoystickConstants.CANCEL_ALL_COMMANDS_O)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        timer.stop();
        timer.reset();
        isRunning = false;
        runningSpeed = 0;
        // CommandScheduler.getInstance().schedule(new MoveRobotOriented(0.25, 0.5, new Rotation2d(0)));
        // if (setType.equals(POSITION_TYPE.LEFT_CONE) || setType.equals(POSITION_TYPE.RIGHT_CONE)) {
        //     CommandScheduler.getInstance().schedule(new AlignWithTape());
        // }
    }

    private void initControllers() {
        xController = new ProfiledPIDController(
            RoutineConstants.TRANSLATION_P, 0, 0, 
            new Constraints(
                RoutineConstants.ROUTINE_MAX_TRANSLATION_SPEED_M_S, 
                RoutineConstants.ROUTINE_MAX_TRANSLATION_ACCEL_M_S_S
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
