package frc.robot.commands;

import org.opencv.core.Point;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Bezier;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPose;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPoseConstraint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.PhotonVision;

public class MoveElevatorToPose extends Command {
    /**
    So the elevator and hand have pose combinations that are illegal and must be avoided
     To avoid this we can set up a x y grid where 
      x represents the height of the elevator in 0 - 1 range
      y represents the angle of the hand in radians
    
    We can represent a group of illegal poses via boxes x1 y1 x2 y2
    We need to pathfind around these boxes while maintaining smooth motion
     To do this, you can:
      1.) Create a smooth bezier curve between points to avoid the boxes
       a.) You can add parameters, such as agressivness (angle)
       b.) You can set the acceptable "distance" from the edge of a box (center control point) 
      2.) Next, you define a time period to complete this in, say 1s
      3.) Then you sample the curve at the timestep to get where the mechanisms should attempt to be at
        (It's worth noting this will be delayed from the curve, so 1s may be ~1.15s in real time)
        (The sampled pose must be agressive enough of a delta from current pose or else it will ocillate)
      4.) If the pose is found to either, leave the safe zone, or be wildly outside, log an error

      This method allows specific avoidance of blockages while maintaining a smooth trajectory
    */ 
    Elevator m_elevator = null;
    Hand m_hand = null;
    ElevatorPose targetPose;
    GenericEntry nte;
    Bezier controlCurve;

    public MoveElevatorToPose(ElevatorPose targetPose, Elevator elevator, Hand hand) {
        m_elevator = elevator;
        m_hand = hand;
        this.targetPose = targetPose;

        addRequirements(m_elevator);
        addRequirements(m_hand);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // woah first time ever using this?
        // So this needs to be fast and initalize the bezier path
        // 1.) Generate the start and end points
        // 2.) Calculate a middle control point and scale curves by agressivness
        
        // then we serialize for visualization in desmos (debug)
        // x\ge6\left\{4<y<8\right\}\left\{x<9\right\} or
        // constraint boxes become 
        // x>=x1{y1<y<y2}{x<x2}
        // bezier points becomes
        // \left(2,3\right),\ \left(3,4\right) or
        // (p1x,p1y), (p2x,p2y)

        // remember
        //   x represents the height of the elevator in 0 - 1 range
        //   y represents the angle of the hand in radians

        controlCurve = new Bezier();

        controlCurve.start = new Point(0, 0);
        controlCurve.end = new Point(.5, Math.PI);
        controlCurve.control = new Point(.25, Math.PI * 1.5);

        // for eve
        Shuffleboard.getTab("elevator").addString("bezier data", () -> getSerialized());
    }

    public String getSerialized() {
        String constraintString = "";
        for (ElevatorPoseConstraint constriant : ElevatorCommandConstants.elevatorConstraints) {
            constraintString += "," + constriant.lowerPoint.elevatorPosition + "," + constriant.lowerPoint.handPosition + "," +
                constriant.upperPoint.elevatorPosition + "," + constriant.upperPoint.handPosition;
        }
        constraintString = constraintString.substring(1); // string leading comma 

        return "\"" + controlCurve.serialize(100) + "\" \"" + constraintString + "\"";
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }


}
