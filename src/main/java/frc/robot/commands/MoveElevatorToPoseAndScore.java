package frc.robot.commands;

import org.opencv.core.Point;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Bezier;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.constants.HandConstants;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPose;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPoseConstraint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.PhotonVision;

public class MoveElevatorToPoseAndScore extends Command {
    Elevator m_elevator = null;
    Hand m_hand = null;
    ElevatorPose targetPose;
    GenericEntry nte;
    Bezier controlCurve;
    boolean passedElevatorCutoff = false; // where it is safe to put it back out to real pose
    Timer timeAtPose = new Timer();

    public MoveElevatorToPoseAndScore(ElevatorPose targetPose, Elevator elevator, Hand hand) {
        m_elevator = elevator;
        m_hand = hand;
        this.targetPose = targetPose;

        addRequirements(m_elevator);
        addRequirements(m_hand);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setElevatorToPosition(targetPose.elevatorPosition);
        m_hand.setWristSetpoint(HandConstants.safePassingAngle);
        timeAtPose.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // TODO: check if within bound and activate actuator
        if (m_hand.atPose()) {
            m_elevator.seekPosition();
            System.out.println("at pose");
            if (m_elevator.atPose()) {
                System.out.println("at elev pose");
                passedElevatorCutoff = true;
                m_hand.setWristSetpoint(targetPose.handPosition);
            }
        }
        m_hand.seekPosition();
    }
    
    @Override
    public boolean isFinished() {
        boolean ended = m_hand.atPose() && passedElevatorCutoff && m_elevator.atPose();
        if (!ended) {
            timeAtPose.restart();
        }
        // System.out.println(ended);
        return ended && timeAtPose.get() > 0.1;
        // return false;
    }

    // @Override
    // public void end(boolean interrupted) {
    //     // TODO Auto-generated method stub
    //     super.end(interrupted);
    // }

}
