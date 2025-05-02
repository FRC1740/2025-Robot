package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.QuestNav;
import frc.robot.constants.VisionConstants;

public class QuestNavSubsystem extends SubsystemBase {
    private static QuestNavSubsystem instance;
    
    static Matrix<N3, N1> QUESTNAV_STD_DEVS =
    VecBuilder.fill(
        0.02, // Trust down to 2cm in X direction
        0.02, // Trust down to 2cm in Y direction
        0.035 // Trust down to 2 degrees rotational
    );
    
    QuestNav questNav;
    CommandSwerveDrivetrain m_drive;
    boolean hasBeenReset;

    public static QuestNavSubsystem getInstance() {
        if(instance == null) {
            instance = new QuestNavSubsystem();
        }
        return instance;
    }
    
    public QuestNavSubsystem() {
        questNav = new QuestNav();
        m_drive = CommandSwerveDrivetrain.getInstance();
        hasBeenReset = false;
    }

    @Override
    public void periodic() {
        questNav.cleanupResponses();
        questNav.processHeartbeat();

        // connection check
        if (questNav.getConnected() && questNav.getTrackingStatus()) {
            // If we haven't reset it, it would skew us way far onto the corner of the field
            if (hasBeenReset) {
                Pose2d pose = getPose(); 
                // Get timestamp from the QuestNav instance
                double timestamp = questNav.getTimestamp();

                // Convert FPGA timestamp to CTRE's time domain using Phoenix 6 utility
                double ctreTimestamp = Utils.fpgaToCurrentTime(timestamp);

                // You can put some sort of filtering here if you would like!

                // Add the measurement to our estimator
                // Importantly, because we reset the headset to a vision pose, it acts as a camera pose would
                m_drive.addVisionMeasurement(pose, ctreTimestamp, QUESTNAV_STD_DEVS);
            }
        }
    }

    /** 
     * @return The robot position on the field, accounting for offcenter questnav
     */
    public Pose2d getPose() {
        Pose2d questPose = questNav.getPose();
        Pose2d robotPose = questPose.transformBy(VisionConstants.QuestToRobot.inverse());
        
        return robotPose;
    }

    public void setPose(Pose2d resetPose) {
        // Transform by the offset to get the Quest pose
        Pose2d questPose = resetPose.transformBy(VisionConstants.QuestToRobot);

        // Send the reset operation
        questNav.setPose(questPose);

        hasBeenReset = true;
    }
}
