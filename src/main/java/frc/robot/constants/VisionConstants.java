package frc.robot.constants;

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public final class VisionConstants {
  //10.17.40.2:5810
  // 10.17.40.11
  // http://photonvision.local:5800/#/dashboard
  public static final String camName = "Cam1"; // grey
  public static final String cam2Name = "Cam2"; // white
  public static final Double AprilTagMinimumArea = 0.0;

  public static final Double cam12Dist = .27; // 10.5in is .27m

  public static final HashMap<String, Transform3d> cameraOffsets = new HashMap<>();
  public static AprilTagFieldLayout aprilTagFieldLayout = null;
  
  static {
    // y = l-r, x f-b
    // offset to center of robot meters
    cameraOffsets.put("Cam1", new Transform3d(0.0, cam12Dist / 2.0, 0.0, new Rotation3d()));
    cameraOffsets.put("Cam2", new Transform3d(0.0, -cam12Dist  / 2.0, 0.0, new Rotation3d()));

    try{
      // if you set this, you may get incorrect tag positions!!!
      // make sure this is updated to the current game
      aprilTagFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile); }
    catch(IOException IOE){
      IOE.printStackTrace();
    }
  }

  public static final Transform3d RobotToCam1 = new Transform3d(0.0, -cam12Dist, 0.0, new Rotation3d());
  public static final Transform3d RobotToCam2 = new Transform3d(0.0, cam12Dist, 0.0, new Rotation3d());


  public enum AprilTagIDs{
    RedSpeakerCenter(4), //Center tag on red speaker
    RedSpeakerSide(3), //Side tag on red speaker
    BlueSpeakerCenter(7), //Center tag on blue speaker
    BlueSpeakerSide(8), //Side tag on blue speaker
    RedSourceDriverStationClose(9), //Red source tag (Closer to blue alliance diver station)
    RedSourceDriverStationFar(10), //Red source tag (Further from blue alliance driver station)
    BlueSourceDriverStationClose(2), //Blue source tag (Closer to red alliance diver station)
    BlueSourceDriverStationFar(1), //Blue source tag (Further from red alliance driver station)
    RedAmp(5), 
    BlueAmp(6);

    private final int ID;

    AprilTagIDs(int ID){
        this.ID = ID;
    }
    public int getID(){
        return ID;
    }
  }
  public static boolean isSpeakerID(int testID){
    if(testID == AprilTagIDs.RedSpeakerCenter.getID() || testID == AprilTagIDs.BlueSpeakerCenter.getID() ||
      testID == AprilTagIDs.RedSpeakerSide.getID() || testID == AprilTagIDs.BlueSpeakerSide.getID()){
      return true;

    }
    return false;
  }
  public static boolean isAmpID(int testID){
    if(testID == AprilTagIDs.RedAmp.getID() || testID == AprilTagIDs.BlueAmp.getID()){
      return true;
    }
    return false;
  }
}
