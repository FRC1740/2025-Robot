package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    SparkBase elevator = null;
    RelativeEncoder elevatorEncoder = null;
    PIDController elevatorController = null;
    public Elevator() {
        elevator = new SparkMax(ElevatorConstants.driveCanId, MotorType.kBrushless);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit);

        elevatorController = new PIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        
        
        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = elevator.getEncoder();
    }

    @Override
    public void periodic() {
        elevator.set(
            elevatorController.calculate(elevator.getEncoder().getPosition()));
    }

    // Moves elevator up to the specified position in inches
    public void setElevatorToPosition(double inches) {
        elevatorController.setSetpoint(inches); // TODO! conversion factor
    }

    // Sets the current position as the "bottom"
    public void resetElevator() {
        
    }
}
