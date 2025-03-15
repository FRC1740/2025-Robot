// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.io.IOException;
import java.util.List;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.RobotContainer;
import frc.robot.constants.DriveCommandConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
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
    double strafe = 0.0;
    Timer timeRunning = new Timer();
    Command pathDrive = null;

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
            Double DriveMaxSpeed, Double DriveMaxAngularRate, CommandXboxController joystick) {
                
        m_joystick = joystick;
        m_drive = CommandSwerveDrivetrain.getInstance();
        m_photonvision = PhotonVision.getInstance();
        MaxSpeed = DriveMaxSpeed;
        MaxAngularRate = DriveMaxAngularRate;
        m_driveRequest = driveRequest;

        this.isLeftReef = leftReef;

        addRequirements(m_drive);
        addRequirements(m_photonvision);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timeRunning.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.err.println("ex");
        PhotonTrackedTarget target = m_photonvision.getBestTarget();
        if (target != null) {
            Optional<Pose3d> tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
            // TODO! check if reef tag
            if (tagPose.isPresent()) {
                targetPose = tagPose.get().toPose2d();
            }
        }

        if (targetPose != null) {
            if (pathDrive == null) {
                distanceToTag = target.bestCameraToTarget; // getTranslationToAprilTag may be incorrect

                double leftToRightOffset = VisionConstants.reefLeftRightOffset;
                if (!isLeftReef) {
                    leftToRightOffset *= -1;
                }

                rotatedGoal = new Pose2d(DriveCommandConstants.xGoal, DriveCommandConstants.yGoal + leftToRightOffset + strafe, new Rotation2d());
                rotatedGoal = rotatedGoal.rotateBy(targetPose.getRotation()); // Rotate the goal to account for rotated tags

                rotatedGoal = new Pose2d(
                        (targetPose.getX() + rotatedGoal.getX()), // apply target offsets
                        (targetPose.getY() + rotatedGoal.getY()),
                        targetPose.getRotation().plus(new Rotation2d(Math.PI))); // normal of the tag is flipped from robot
                                                                                // target
                PosePublisher.set(new Pose2d[] { rotatedGoal });

                List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
                    m_drive.getState().Pose,
                    rotatedGoal
                );

                PathConstraints constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
                // PathConstraints constraints = PathConstraints.unlimitedConstraints(12.0); // You can also use unlimited constraints, only limited by motor torque and nominal battery voltage

                // Create the path using the waypoints created above
                PathPlannerPath path = new PathPlannerPath(
                        waypoints,
                        constraints,
                        null, // The ideal starting state, this is only relevant for pre-planned paths, so can be null for on-the-fly paths.
                        new GoalEndState(0.0, targetPose.getRotation()) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
                );

                // Prevent the path from being flipped if the coordinates are already correct
                path.preventFlipping = true;
                m_drive.configureAutoBuilder();
                try {
                    pathDrive = new FollowPathCommand(
                            path,
                            () ->  m_drive.getState().Pose, // Robot pose supplier
                            () ->  m_drive.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                            (speeds, feedforwards) -> m_drive.setControl(
                                m_drive.m_pathApplyRobotSpeeds.withSpeeds(speeds)
                                    .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                    .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                            ),
                            new PPHolonomicDriveController(
                                // PID constants for translation
                                new PIDConstants(10, 0, 0),
                                // PID constants for rotation
                                new PIDConstants(7, 0, 0)
                            ),
                            RobotConfig.fromGUISettings(),
                            // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                            m_drive // Reference to this subsystem to set requirements
                    );
                    pathDrive.initialize();
                } catch (Exception ex) {
                    DriverStation.reportError("Failed to load PathPlanner config and configure On The Fly Path", ex.getStackTrace());
                }
            }else {
                pathDrive.execute();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        pathDrive.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return XFinished && YFinished && ThetaFinished;
        return false; // strafe
    }
}
