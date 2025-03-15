package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

public class Intake extends Command {
    Elevator m_elevator = null;
    Hand m_hand = null;
    GenericEntry nte;
    // boolean setElevatorHeight = false;

    public Intake() {
        m_elevator = Elevator.getInstance();
        m_hand = Hand.getInstance();

        addRequirements(m_elevator);
        addRequirements(m_hand);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_elevator.setElevatorToPosition(m_elevator.getElevatorPosition());
        m_hand.setWristSetpoint(ElevatorCommandConstants.Intake.handPosition);
        m_hand.intake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // TODO: check if within bound and activate actuator
            // System.out.println("notat pose");
        m_elevator.seekPosition();
        if (m_hand.atPose()) {
            // if (!setElevatorHeight) {
            m_elevator.setElevatorToPosition(ElevatorCommandConstants.Intake.elevatorPosition);
            //     setElevatorHeight = true;
            // }
            
            // System.out.println("2at pose");
            if (m_elevator.atPose()) {
                // System.out.println("at pose");
                
                // if (m_hand.getLinearActuatorCurrent() > HandConstants.linearActuatorCurrentLimit - 1) {
                //     m_hand.stop();
                // }
            }
        }
        m_hand.seekPosition();
    }
    
    @Override
    public boolean isFinished() {
        return m_hand.getIfHaveCoral();
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_hand.stop();
        super.end(interrupted);
    }

}
