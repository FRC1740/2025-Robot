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

import edu.wpi.first.math.Pair;
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
import frc.robot.CoDriverControl;
import frc.robot.CoDriverControl.CoDriverInput;
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
    Timer timeRunning = new Timer();
    Pathfinding pathfinder = null;
    Command pathDrive = null;
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
            Double DriveMaxSpeed, Double DriveMaxAngularRate, CommandXboxController joystick) {
                
        m_joystick = joystick;
        m_drive = CommandSwerveDrivetrain.getInstance();
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
        pathfinder = null;
        timeRunning.start();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        PathConstraints constraints = new PathConstraints(8.0, 3.0, 4 * Math.PI, 6 * Math.PI); // The constraints for this path.

        if (targetPose == null) {
            Optional<Pose3d> tagPose = null;
            switch (m_photonvision.selectedPosition) {
                case A:
                case B:
                System.out.println("AB");
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(18);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(7);
                    }
                    m_photonvision.targetingLeftReef = (m_photonvision.selectedPosition == CoDriverInput.A);
                    break;
                case C:
                case D:
                System.out.println("C");
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(17);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(8);
                    }
                    m_photonvision.targetingLeftReef = (m_photonvision.selectedPosition == CoDriverInput.C);
                    break;
                case E:
                case F:
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(22);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(9);
                    }
                    m_photonvision.targetingLeftReef = (m_photonvision.selectedPosition == CoDriverInput.E);
                    break;
                case G:
                case H:
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(21);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(10);
                    }
                    m_photonvision.targetingLeftReef = (m_photonvision.selectedPosition == CoDriverInput.G);
                    break;
                case I:
                case J:
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(20);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(11);
                    }
                    m_photonvision.targetingLeftReef = (m_photonvision.selectedPosition == CoDriverInput.I);
                    break;
                case K:
                case L:
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(19);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(6);
                    }
                    m_photonvision.targetingLeftReef = (m_photonvision.selectedPosition == CoDriverInput.K);
                    break;
                case LeftSource:
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(13);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(1);
                    }
                    break;
                case RightSource:
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(12);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(2);
                    }
                    break;
                    
                default:
                break;

            }
            // Random rand = new Random();
            // tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(rand.nextInt(6, 11+1)); // 17-22 blue + 1-2 + 6-11 red

            isLeftReef = !m_photonvision.targetingLeftReef;

            // TODO! check if reef tag
            if (tagPose.isPresent()) {
                targetPose = tagPose.get().toPose2d();
            }
        }else {
            if (pathfinder == null) {

                double leftToRightOffset = VisionConstants.reefLeftRightOffset;
                if (!isLeftReef) {
                    leftToRightOffset *= -1;
                }
                leftToRightOffset += VisionConstants.reefAlignmentFudge;

                double L4Offset = 0.0;

                if (CoDriverControl.getInstance().atL4()) {
                    L4Offset = VisionConstants.reefL4Offset;
                }

                rotatedGoal = new Pose2d(DriveCommandConstants.x2Goal + L4Offset, DriveCommandConstants.yGoal + leftToRightOffset, new Rotation2d());
                if (finishedFirstPath) {
                    rotatedGoal = new Pose2d(DriveCommandConstants.xGoal + L4Offset, DriveCommandConstants.yGoal + leftToRightOffset, new Rotation2d());
                }
                
                rotatedGoal = rotatedGoal.rotateBy(targetPose.getRotation()); // Rotate the goal to account for rotated tags

                rotatedGoal = new Pose2d(
                        (targetPose.getX() + rotatedGoal.getX()), // apply target offsets
                        (targetPose.getY() + rotatedGoal.getY()),
                        targetPose.getRotation().plus(new Rotation2d(Math.PI))); // normal of the tag is flipped from robot
                                                                                // target
                PosePublisher.set(new Pose2d[] { rotatedGoal });

                pathfinder = new Pathfinding();

                Pathfinding.ensureInitialized();
                Pathfinding.setGoalPosition(rotatedGoal.getTranslation());
                Pathfinding.setStartPosition(m_drive.getState().Pose.getTranslation());
            }


            if (pathDrive != null) { 
                if(pathDrive.isFinished()) {
                    pathfinder = null;
                    if (finishedFirstPath) { // finished fr
                        finished = true;
                    }
                    finishedFirstPath = true;
                    pathDrive = null;
                }  
            }

            if (Pathfinding.isNewPathAvailable() && !finished && pathfinder != null) {

                if (pathDrive != null) { 

                    pathDrive.end(false);
                }

                // megatag
                // try hori scoring

                // add   public void setDynamicObstacles(
                //   List<Pair<Translation2d, Translation2d>> obs, Translation2d currentRobotPos) {
                // ONLY ON FIRST SCORING??? (good idea?)

                // maybe reduce stop start jank?
                // double endSpeedGoal = 0.0;
                // if (!finishedFirstPath) {
                //     endSpeedGoal = .5;
                // }
                PathPlannerPath path = Pathfinding.getCurrentPath(constraints, new GoalEndState(0.0, rotatedGoal.getRotation()));
                if (path != null) {
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
                        config,
                        // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                        () -> false,
                        m_drive // Reference to this subsystem to set requirements
                    );

                    pathDrive.initialize();
                }else {
                    // need to reset because failed config
                    pathfinder = null;
                    pathDrive = null;
                }
            }
            if (pathDrive != null && !finished) {
                pathDrive.execute();
            }
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (pathDrive != null) {
            pathDrive.cancel();
            pathDrive = null;
            pathfinder = null;
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // return XFinished && YFinished && ThetaFinished;
        return finished; // strafe
    }
}
