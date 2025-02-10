package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CanIds;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    SparkBase elevator = null;
    RelativeEncoder elevatorEncoder = null;
    PIDController elevatorController = null;
    ShuffleboardTab elevatorTab = null;

    public Elevator() {
        elevator = new SparkMax(CanIds.elevatorCanId, MotorType.kBrushless);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit);

        elevatorController = new PIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        
        
        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = elevator.getEncoder();

        elevatorTab = Shuffleboard.getTab("elevator");
        elevatorTab.addFloat("elevator position", () -> (float)getElevatorPosition());
        elevatorTab.addFloat("elevator setpoint", () -> (float)getElevatorSetpoint());
    }

    @Override
    public void periodic() {}

    /**
     * Runs one step to optimize the PID and get new outputs for the inputs
     */
    public void seekPosition() {
        elevator.set(
            elevatorController.calculate(elevatorEncoder.getPosition()) * ElevatorConstants.outputFactor);
        // System.out.println(elevatorController.calculate(elevatorEncoder.getPosition()) * ElevatorConstants.outputFactor);
    }

    /**
     * Set the elevator setpoint up to the specified position in inches 
     * @param inches distance from bottom
     */
    public void setElevatorToPosition(double inches) {
        elevatorController.setSetpoint(inches); // TODO! conversion factor
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public double getElevatorPosition() {
        return elevatorEncoder.getPosition(); // TODO! conversion factor
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public double getElevatorSetpoint() {
        return elevatorController.getSetpoint(); // TODO! conversion factor
    }

    /**
     * Sets the current position as the "bottom"
     */
    public void resetElevator() {
        elevatorEncoder.setPosition(0.0);
    }
}
