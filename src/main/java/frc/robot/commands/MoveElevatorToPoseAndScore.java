package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPose;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

public class MoveElevatorToPoseAndScore extends Command {
    Elevator m_elevator = null;
    Hand m_hand = null;
    ElevatorPose targetPose;
    GenericEntry nte;
    boolean passedElevatorCutoff = false; // where it is safe to put it back out to real pose
    Timer timeAtPose = new Timer();

    // TODO! make scoring tap height, score and then release when scroed to go to pos

    public MoveElevatorToPoseAndScore(ElevatorPose targetPose) {
        m_elevator = Elevator.getInstance();
        m_hand = Hand.getInstance();
        this.targetPose = targetPose;

        addRequirements(m_elevator);
        addRequirements(m_hand);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setElevatorToPosition(m_elevator.getElevatorPosition()); // hold current
        m_hand.setWristSetpoint(targetPose.handPosition);
        timeAtPose.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // TODO: check if within bound and activate actuator
        m_elevator.seekPosition();
        
        if (m_hand.atPose()) {
            m_elevator.setElevatorToPosition(targetPose.elevatorPosition);
            // System.out.println("at pose");
            if (m_elevator.atPose()) {
                // System.out.println("at elev pose");
                passedElevatorCutoff = true;
                m_hand.setWristSetpoint(targetPose.handPosition);
            }
        }
        m_hand.seekPosition();
    }
    
    @Override
    public boolean isFinished() {
        if (RobotBase.isSimulation()) {
            return true;
        }
        boolean ended = m_hand.atPose() && passedElevatorCutoff && m_elevator.atPose();
        if (!ended) {
            timeAtPose.restart();
        }
        // System.out.println(ended);
        return ended && timeAtPose.get() > 0.3;
        // return false;
    }

    // @Override
    // public void end(boolean interrupted) {
    //     // TODO Auto-generated method stub
    //     super.end(interrupted);
    // }

}
