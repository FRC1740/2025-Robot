package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

public class L4CoralTap extends Command {
    Elevator m_elevator = null;
    Hand m_hand = null;
    GenericEntry nte;
    Timer tapTimer = new Timer();

    public L4CoralTap(Hand hand) {
        m_hand = hand;

        // addRequirements(m_hand); NOT INCLUDED INTENTIONALLY
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        tapTimer.restart();
        m_hand.intake();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }
    
    @Override
    public boolean isFinished() {
        return tapTimer.get() > .1;
    }

    @Override
    public void end(boolean interrupted) {
        m_hand.stop();
        super.end(interrupted);
    }

}
