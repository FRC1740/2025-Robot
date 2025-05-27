package frc.robot.constants;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {
    // 10.17.40.2:5810
    // 10.17.40.11
    // http://photonvision.local:5800/#/dashboard
    public static final String camName = "Cam1"; // grey
    public static final String cam2Name = "Cam2"; // white
    public static final String cam3Name = "Cam3"; // back
    public static final Double AprilTagMinimumArea = 0.0;

    public static final Double reefL4Offset = 0.17;  // How far back to go to score L4
    public static final Double reefLeftRightOffset = 0.15;  // for scoring
    public static final Double reefAlignmentFudge = -0.02;

    public static final Double cam12FrontBackOffset = 0.2; // 8in is .2m
    public static final Double cam12Dist = 0.22; // 10.5in is .27m

    public static AprilTagFieldLayout aprilTagFieldLayout = null;

    static {
        try {
            // if you set this, you may get incorrect tag positions!!!
            // make sure this is updated to the current game
            aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
        } catch (IOException IOE) {
            IOE.printStackTrace();
        }
    }

    public static final Transform3d RobotToCam1 = new Transform3d(cam12FrontBackOffset, -cam12Dist, 0.0, new Rotation3d(0.0, 0.0, -.0));
    
    public static final Transform3d RobotToCam2 = new Transform3d(cam12FrontBackOffset, cam12Dist, 0.0, new Rotation3d(0.0, 0.0, .0));

    // TODO
    public static final Transform3d RobotToCam3 = new Transform3d(-0.3, -.08, 0.0, new Rotation3d(0.0, 0.0, Math.PI));

    public static final Transform2d QuestToRobot = new Transform2d( /*TODO: Put x, y, rotational offsets here!*/ );

    public static final double questVisionUpdateThreshold = 0.1; // TODO! tune

    public enum AprilTagIDs {;

        private final int ID;

        AprilTagIDs(int ID) {
            this.ID = ID;
        }

        public int getID() {
            return ID;
        }
    }
}
