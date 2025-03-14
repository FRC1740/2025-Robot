package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CanIds;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase {
    SparkBase climber = null; // rotates to pos

    private static Climber instance;

    public static Climber getInstance() {
      if(instance == null) {
        instance = new Climber();
      }
      return instance;
    }
    
    public Climber() {
        climber = new SparkMax(CanIds.climberCanId, MotorType.kBrushless);
        SparkMaxConfig wristConfig = new SparkMaxConfig();
        
        wristConfig.smartCurrentLimit(ClimberConstants.climberCurrentLimit);

        wristConfig.idleMode(IdleMode.kBrake);
        
        climber.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    /**
     * Runs climber out
     */
    public void unclimb() {
        climber.set(1.0);
    }
    /**
     * Stops climbing
     */
    public void stop() {
        climber.set(0.0);
    }
    /**
     * Runs climber in
     */
    public void climb() {
        climber.set(-.75);
    }
}
