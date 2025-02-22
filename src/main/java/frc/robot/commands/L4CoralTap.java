package frc.robot.commands;

import org.opencv.core.Point;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Bezier;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.HandConstants;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPose;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPoseConstraint;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.PhotonVision;

public class L4CoralTap extends Command {
    Elevator m_elevator = null;
    Hand m_hand = null;
    GenericEntry nte;
    Bezier controlCurve;
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
