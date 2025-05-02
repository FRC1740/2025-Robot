// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.List;
import java.util.Optional;
import java.util.Random;

import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.CoDriverControl.CoDriverInput;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveCommandConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Pathfind;
import frc.robot.subsystems.PhotonVision;

public class AlignToTagPathplanner extends Command {
    CommandSwerveDrivetrain m_drive;
    private PhotonVision m_photonvision;

    double x_error;
    double y_error;
    double theta_error;
    double angleToTag;
    Transform3d distanceToTag;
    boolean XFinished;
    boolean YFinished;
    boolean ThetaFinished;
    boolean isLeftReef; // A C E etc
    Pose2d targetPose = null;
    Pose2d rotatedGoal = null;
    Double MaxSpeed = null;
    Double MaxAngularRate = null;
    SwerveRequest.FieldCentric m_driveRequest;
    CommandXboxController m_joystick = null;
    Timer timeRunning = new Timer();
    Pathfinding pathfinder = null;
    Command pathDrive = null;
    Pathfind m_pathfind;
    RobotConfig config = null;
    boolean finishedFirstPath = false;
    boolean finished = false;

    NetworkTable DriveTrainTable = NetworkTableInstance.getDefault().getTable("DriveTrain");

    // Pose data Publisher
    StructArrayPublisher<Pose2d> PosePublisher = DriveTrainTable
            .getStructArrayTopic("Target Pose", Pose2d.struct).publish();

    public static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    /**
     * Creates a new Command that aligns the robot angle to an apriltag using the
     * Limelight.
     * <br>
     * </br>
     * This command <b>DOES DRIVE</b>
     */
    public AlignToTagPathplanner(boolean leftReef, SwerveRequest.FieldCentric driveRequest, 
            Double DriveMaxSpeed, Double DriveMaxAngularRate, CommandXboxController joystick, Pathfind pathfind) {
                
        m_joystick = joystick;
        m_drive = CommandSwerveDrivetrain.getInstance();
        m_pathfind = pathfind;
        m_photonvision = PhotonVision.getInstance();
        MaxSpeed = DriveMaxSpeed;
        MaxAngularRate = DriveMaxAngularRate;
        m_driveRequest = driveRequest;
        finishedFirstPath = false;

        this.isLeftReef = leftReef;
            
        try {    
            config = RobotConfig.fromGUISettings();
        }catch (Exception ex) {

        }

        addRequirements(m_drive);
        addRequirements(m_photonvision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        finished = false;
        finishedFirstPath = false;
        targetPose = null;
        timeRunning.start();
        m_pathfind.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        Command path = m_pathfind.getPath();
        if (path != null) {
            path.execute();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return XFinished && YFinished && ThetaFinished;
        return finished; // strafe
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_pathfind.end();
    }
}
