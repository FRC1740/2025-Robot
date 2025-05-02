package frc.robot.subsystems;

import java.util.Optional;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.Pathfinding;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.CoDriverControl.CoDriverInput;
import frc.robot.constants.DriveCommandConstants;
import frc.robot.constants.VisionConstants;

public class Pathfind extends SubsystemBase {

    private static Pathfind instance;
    private PhotonVision m_photonvision;
    Pathfinding pathfinder = null;
    Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(0));
    CommandSwerveDrivetrain m_drive;
    Command pathDrive = null;
    Pose2d rotatedGoal = null;
    RobotConfig config = null;
    CoDriverInput selectedPosition;
    boolean close = false;
    boolean running = false;
    Timer recalcTimer;
    double recalcTime = 1.0;
    PathConstraints constraints = new PathConstraints(3.0, 1.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.

    public static Pathfind getInstance() {
      if(instance == null) {
        instance = new Pathfind();
      }
      return instance;
    }

    public Pathfind() {
        Pathfinding.ensureInitialized();
        m_photonvision = PhotonVision.getInstance();
        m_drive = CommandSwerveDrivetrain.getInstance();
        try {    
            config = RobotConfig.fromGUISettings();
        }catch (Exception ex) {

        }
        setScoringPosition(CoDriverInput.A);
        recalcTimer = new Timer();
        recalcTimer.start();
    }
    public void setScoringPosition(CoDriverInput selectedPosition) {
        setScoringPosition(selectedPosition, false);
    }

    public void setScoringPosition(CoDriverInput selectedPosition, boolean close) {
            if (!running) {
                close = false;
            }
            this.selectedPosition = selectedPosition;
            this.close = close;
            Optional<Pose3d> tagPose = null;

            switch (selectedPosition) {
                case A:
                case B:
                    if (!m_drive.m_operatorPerspectiveFlipped) { // blue
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(18);
                    }else {
                        tagPose = VisionConstants.aprilTagFieldLayout.getTagPose(7);
                    }
                    m_photonvision.targetingLeftReef = (selectedPosition == CoDriverInput.A);
                    break;
                case C:
                case D:
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

            // TODO! check if reef tag
            if (tagPose.isPresent()) {
                targetPose = tagPose.get().toPose2d();
            }

            double leftToRightOffset = VisionConstants.reefLeftRightOffset;
            if (!m_photonvision.targetingLeftReef) {
                leftToRightOffset *= -1;
            }
            leftToRightOffset += VisionConstants.reefAlignmentFudge;

            rotatedGoal = new Pose2d(DriveCommandConstants.x2Goal, DriveCommandConstants.yGoal + leftToRightOffset, new Rotation2d());
            if (close) {
                rotatedGoal = new Pose2d(DriveCommandConstants.xGoal, DriveCommandConstants.yGoal + leftToRightOffset, new Rotation2d());
            }
            rotatedGoal = rotatedGoal.rotateBy(targetPose.getRotation()); // Rotate the goal to account for rotated tags

            rotatedGoal = new Pose2d(
                    (targetPose.getX() + rotatedGoal.getX()), // apply target offsets
                    (targetPose.getY() + rotatedGoal.getY()),
                    targetPose.getRotation().plus(new Rotation2d(Math.PI))); // normal of the tag is flipped from robot
                                                                            // target
            // PosePublisher.set(new Pose2d[] { rotatedGoal });

            pathfinder = new Pathfinding();

            Pathfinding.ensureInitialized();
            Pathfinding.setGoalPosition(rotatedGoal.getTranslation());
            Pathfinding.setStartPosition(m_drive.getState().Pose.getTranslation());
    }

    @Override
    public void periodic() {
        if (pathDrive != null) {
            if (pathDrive.isFinished()) {
                setScoringPosition(selectedPosition, true);
            }
        }
        if (Pathfinding.isNewPathAvailable()) {
            
                if (pathDrive != null) { 

                    pathDrive.end(false);
                }

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
        if (!running && recalcTimer.get() > recalcTime) {
            setScoringPosition(selectedPosition, close);
            recalcTimer.restart();
        }
    }

    public Command getPath() {
        return pathDrive;
    }

    public void start() {
        running = true;
    }
    public void end() {
        running = false;
        close = false;
    }
}
