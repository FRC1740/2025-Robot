package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

public class Score extends Command {
    Elevator m_elevator = null;
    Hand m_hand = null;
    GenericEntry nte;
    Timer runTime = new Timer();

    public Score() {
        m_hand = Hand.getInstance();
        addRequirements(m_hand);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_hand.score();
        runTime.restart();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }
    
    @Override
    public boolean isFinished() {
        return !m_hand.getIfHaveCoral() && runTime.get() > .3;
    }

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        m_hand.stop();
        super.end(interrupted);
    }

}
