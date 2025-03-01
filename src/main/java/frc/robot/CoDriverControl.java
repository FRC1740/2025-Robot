package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.Intake;
import frc.robot.commands.L4CoralTap;
import frc.robot.commands.MoveElevatorToPoseAndScore;
import frc.robot.commands.Score;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;

public class CoDriverControl {
    public enum CoDriverInput {
        L0,
        L1,
        L2,
        L3,
        L4,
        L2Algae,
        L3Algae,
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L,
    }
    
    public CoDriverInput lastCoDriverInput = null;
    public Command elevatorControl = new InstantCommand();
    Elevator m_elevator = null; 
    Hand m_hand = null;
      
    
    public CoDriverControl(Elevator elevator, Hand hand) {
        m_elevator = elevator;
        m_hand = hand;
    }

    public void sendInput(CoDriverInput input) {
        if (input == null) {
            lastCoDriverInput = null;
            return;
        }else {
            if (input == CoDriverInput.L1 || input == CoDriverInput.L2 || 
                input == CoDriverInput.L3 || input == CoDriverInput.L4) {
                if (lastCoDriverInput == input) { // double tap to score
                    elevatorControl.cancel();
                    elevatorControl =  new Score(m_elevator, m_hand);
                    System.out.println("yeah");
                }else { // tap to raise to height
                    elevatorControl.cancel();
                    switch (input) {
                        case L1:
                            elevatorControl = new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L1Score, m_elevator, m_hand);
                            break;
                        case L2:
                            elevatorControl = new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L2Score, m_elevator, m_hand);
                            break;
                        case L3:
                            elevatorControl = new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L3Score, m_elevator, m_hand);
                            break;
                        case L4:
                            elevatorControl = new SequentialCommandGroup(
                                new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L4Score, m_elevator, m_hand),
                                new L4CoralTap(m_hand)
                            );
                            break;
                    
                        default:
                            break;
                    }
                }

                elevatorControl.schedule();
            }else if(input == CoDriverInput.L0){
                elevatorControl.cancel();
                elevatorControl = new Intake(m_elevator, m_hand);
                elevatorControl.schedule();
            }else if (input == CoDriverInput.L2Algae) {
                elevatorControl.cancel();
                if (lastCoDriverInput == input) { // double tap to score
                    elevatorControl = new InstantCommand(() -> {
                        m_elevator.setElevatorToPosition(m_elevator.targetPosition - 1.0);
                    });
                }else {
                    elevatorControl = new ParallelCommandGroup(
                        new InstantCommand(() -> {
                            m_hand.score();
                        }),
                        new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L2Algae, m_elevator, m_hand)
                    );
                }
                elevatorControl.schedule();
            }else if (input == CoDriverInput.L3Algae) {
                elevatorControl.cancel();
                if (lastCoDriverInput == input) { // double tap to score
                    elevatorControl = new InstantCommand(() -> {
                        m_elevator.setElevatorToPosition(m_elevator.targetPosition - 1.0);
                    });
                }else {
                    elevatorControl = new ParallelCommandGroup(new InstantCommand(() -> {
                        m_hand.score();
                    }),
                        new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L3Algae, m_elevator, m_hand)
                    );   
                }
                elevatorControl.schedule();
            }
            lastCoDriverInput = input;
        }
    }
}
