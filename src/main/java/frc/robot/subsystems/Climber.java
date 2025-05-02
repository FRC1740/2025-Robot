package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Telemetry;
import frc.robot.constants.CanIds;
import frc.robot.constants.ClimberConstants;

public class Climber extends SubsystemBase {
    SparkBase climber = null; // rotates to pos
    AbsoluteEncoder encoder = null;
    SparkBase climberWheel = null; // rotates

    Telemetry m_telemetry = null;

    private static Climber instance;

    public static Climber getInstance() {
      if(instance == null) {
        instance = new Climber();
      }
      return instance;
    }
    
    public Climber() {
        m_telemetry = Telemetry.getInstance();
        climber = new SparkMax(CanIds.climberCanId, MotorType.kBrushless);
        SparkMaxConfig climberConfig = new SparkMaxConfig();
        
        climberConfig.smartCurrentLimit(ClimberConstants.climberCurrentLimit);

        climberConfig.idleMode(IdleMode.kBrake);
        
        climber.configure(climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        encoder = climber.getAbsoluteEncoder();

        climberWheel = new SparkMax(CanIds.climberWheelCanId, MotorType.kBrushless);
        SparkMaxConfig climberWheelConfig = new SparkMaxConfig();
        
        climberWheelConfig.smartCurrentLimit(ClimberConstants.climberWheelCurrentLimit);

        climberWheelConfig.idleMode(IdleMode.kBrake);
        
        climber.configure(climberWheelConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        m_telemetry.telemeterizeElevator(getPosition(), getWheelCurrentDraw());
    }

    /**
     * Runs climber out
     */
    public void unclimb() {
        climber.set(1.0);
        climberWheel.set(.75);
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
        climber.set(-1.0);
        climberWheel.set(0.0);
    }

    /**
     * @return climber position
     */
    public double getPosition() {
        return encoder.getPosition();
    }

    /**
     * @return wheel current draw
     */
    public double getWheelCurrentDraw() {
        return climberWheel.getOutputCurrent();
    }
}
