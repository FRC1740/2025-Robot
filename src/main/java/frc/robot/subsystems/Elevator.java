package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.AlternateEncoderConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CanIds;
import frc.robot.constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
    private static double kDt = 0.02;
    SparkBase elevator = null;
    RelativeEncoder elevatorEncoder = null;
    PIDController elevatorController = null;
    private final TrapezoidProfile m_profile =
        new TrapezoidProfile(new TrapezoidProfile.Constraints(1000.0, 10));
    private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
    private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();
    ShuffleboardTab elevatorTab = null;
    public double targetPosition = 0.0;

    public Elevator() {
        elevator = new SparkMax(CanIds.elevatorCanId, MotorType.kBrushless);
        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.encoder.positionConversionFactor(ElevatorConstants.elevatorConversionFactor);
        elevatorConfig.softLimit.forwardSoftLimitEnabled(true);
        elevatorConfig.softLimit.forwardSoftLimit(0.0);
        elevatorConfig.softLimit.reverseSoftLimitEnabled(true);
        elevatorConfig.softLimit.reverseSoftLimit(-28.0);
        elevatorConfig.smartCurrentLimit(ElevatorConstants.elevatorCurrentLimit);

        elevatorController = new PIDController(
            ElevatorConstants.kP, ElevatorConstants.kI, ElevatorConstants.kD);
        
        
        elevator.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        elevatorEncoder = elevator.getEncoder();

        elevatorTab = Shuffleboard.getTab("elevator");
        elevatorTab.addFloat("elevator position2", () -> (float)getElevatorPosition());
        elevatorTab.addFloat("elevator setpoint", () -> (float)getElevatorSetpoint());

        elevatorTab.addFloat("elevator current draw", () -> (float)elevator.getOutputCurrent());
        elevatorTab.addBoolean("at pose", () -> atPose());
    }

    @Override
    public void periodic() {
    }

    /**
     * Runs one step to optimize the PID and get new outputs for the inputs
     */
    public void seekPosition() {
        // double output = elevatorController.calculate(elevatorEncoder.getPosition());
        // if (output > 0.0) {
        //     output /= 6.0;
        //     output -= 0.1;
        //     output = Math.max(output, 0.0);
        // }
        // elevator.set(
        //     output);
        m_setpoint = m_profile.calculate(kDt, m_setpoint, m_goal);
        elevatorController.setSetpoint(m_setpoint.position);
        double output = elevatorController.calculate(elevatorEncoder.getPosition());
        elevator.set(output);

        // System.out.println(elevatorController.calculate(elevatorEncoder.getPosition()) * ElevatorConstants.outputFactor);
    }

    /**
     * Set the elevator setpoint up to the specified position in inches 
     * @param inches distance from bottom
     */
    public void setElevatorToPosition(double inches) {
        elevatorController.setSetpoint(inches);
        m_goal = new TrapezoidProfile.State(inches, 0);
        targetPosition = inches;
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public double getElevatorPosition() {
        return elevatorEncoder.getPosition();
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public boolean atPose() {
        return Math.abs(getElevatorPosition() - getElevatorSetpoint()) < 5.0;
    }
    /**
     * Gets the current elevator position
     * @return the elevator position
     */
    public double getElevatorSetpoint() {
        return elevatorController.getSetpoint();
    }

    /**
     * Sets the current position as the "bottom"
     */
    public void resetElevator() {
        elevatorEncoder.setPosition(0.0);
    }
}
