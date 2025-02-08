package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CanIds;
import frc.robot.constants.HandConstants;

public class Hand extends SubsystemBase {
    SparkBase wrist = null; // rotates to pos
    PIDController wristController = null;
    AbsoluteEncoder wristEncoder = null;

    SparkBase linearActuator = null; // intake / outtakes, no pid just run or no run
    
    public Hand() {
        wrist = new SparkMax(HandConstants.wristCanId, MotorType.kBrushed);
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        
        wristConfig.smartCurrentLimit(HandConstants.wristCurrentLimit);

        wristController = new PIDController(
            HandConstants.kWristP, HandConstants.kWristI, HandConstants.kWristD);
        
        wrist.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        wristEncoder = wrist.getAbsoluteEncoder();


        linearActuator = new SparkMax(CanIds.linearActuatorCanId, MotorType.kBrushed);

        SparkMaxConfig linearActuatorConfig = new SparkMaxConfig();

        linearActuatorConfig.smartCurrentLimit(HandConstants.linearActuatorCurrentLimit);

        linearActuator.configure(linearActuatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs one step to optimize the PID and get new outputs for the inputs
     */
    public void seekPosition() {
        wrist.set(
            wristController.calculate(wristEncoder.getPosition() * 2 * Math.PI));
    }

    /**
     * Sets setpoint for wrist to go to specified radian angle
     * @param angle
     */
    public void rotateWristToPosition(double angle) {
        wristController.setSetpoint(angle);
    }

    /**
     * Runs linear actuator in
     */
    public void intake() {
        linearActuator.set(-1.0);
    }
    /**
     * Runs linear actuator out
     */
    public void score() {
        linearActuator.set(1.0);
    }
}
