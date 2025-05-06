package frc.robot.commands;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.ClimberConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

public class AutoClimb extends Command {

    Climber m_climber;
    boolean extended;
    boolean hooked;
    Timer hookedTimer;
    CommandSwerveDrivetrain m_drive;

    public AutoClimb() {
        m_climber = Climber.getInstance();
        m_drive = CommandSwerveDrivetrain.getInstance();
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        extended = false;
        hooked = false;
        hookedTimer = new Timer();
        hookedTimer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!extended) {
            m_climber.unclimb();
            if (m_climber.getPosition() < ClimberConstants.extendedPosition * 5 || m_climber.getPosition() > 1.0 - ClimberConstants.extendedPosition) {
                extended = true;
                m_climber.stop(); // wheel stays spinning
            }
        }else {
            // TODO! this probably needs rolling average or a timer
            if (m_climber.getWheelCurrentDraw() > ClimberConstants.latchedCurrentDraw) {
                hookedTimer.start();
                // System.out.println("started timer");
            }else {
                hookedTimer.stop();
                hookedTimer.reset();
            }

            if (hookedTimer.get() > .5) {
                hooked = true;
            }

            if (hooked) {
                System.out.println("climbing");
                if (m_climber.getPosition() > ClimberConstants.climbedPosition) {
                    m_climber.climb();
                }else {
                    m_climber.stop();
                }
            }
        }
    }
    
    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        m_climber.stop();
    }

}
