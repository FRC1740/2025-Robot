// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.CoDriverControl.CoDriverInput;
import frc.robot.commands.AlignToTagPathplanner;
import frc.robot.commands.AlignToTagPose;
import frc.robot.commands.AlignToTagPoseHelp;
import frc.robot.commands.AlignToTagSimpleLimelight;
import frc.robot.commands.Intake;
import frc.robot.commands.L4CoralTap;
import frc.robot.commands.MoveElevatorToPoseAndScore;
import frc.robot.commands.Score;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Hand;
import frc.robot.subsystems.PhotonVision;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            // .withDeadband(MaxSpeed * 0.03).withRotationalDeadband(MaxAngularRate * 0.03) // Add a 3% deadband to motor out
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final SwerveRequest.RobotCentricFacingAngle commandDrive = new SwerveRequest.RobotCentricFacingAngle()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry m_logger = Telemetry.getInstance();

    public final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController coDriverController1 = new CommandXboxController(1);
    private final CommandXboxController coDriverController2 = new CommandXboxController(2);

    public final CommandSwerveDrivetrain m_drivetrain = CommandSwerveDrivetrain.getInstance(); 
    public final Elevator m_elevator = Elevator.getInstance();
    public final Hand m_hand = Hand.getInstance();
    public final Climber m_climber = Climber.getInstance();
    public CoDriverControl m_coDriverControl = CoDriverControl.getInstance();

    public final PhotonVision photonvision = PhotonVision.getInstance();

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    private static RobotContainer instance;

    public static RobotContainer getInstance() {
        if(instance == null) {
            instance = new RobotContainer();
        }
        return instance;
    }

    public RobotContainer() {
        NamedCommands.registerCommand("L3", new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L3Score));
        NamedCommands.registerCommand("L4", new SequentialCommandGroup(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L4Score),
            new L4CoralTap()
        ));
        NamedCommands.registerCommand("Intake Pos", new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Intake));
        NamedCommands.registerCommand("Intake", new Intake());
        NamedCommands.registerCommand("Score", new Score());
        joystick.setRumble(RumbleType.kBothRumble, 0.0);
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    double driveCurve(double input) {
        double minInput = .03;
        return (((input * input) + minInput) - (input * minInput));
    }

    double turnCurve(double input) {
        double minInput = .1;
        return (((input * input) + minInput) - (input * minInput));
    }

    int inputLessThanDeadband(double input, double deadband) {
        if (Math.abs(input) < deadband) {
            return 0;
        }
        return (int)Math.signum(input);
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(
                        -driveCurve(Math.abs(joystick.getLeftY())) * 
                            inputLessThanDeadband(joystick.getLeftY(), 0.03) * 
                            MaxSpeed
                    ) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -driveCurve(Math.abs(joystick.getLeftX())) * 
                        inputLessThanDeadband(joystick.getLeftX(), 0.03) * 
                        MaxSpeed
                    ) // Drive left with negative X (left)
                    .withRotationalRate(
                        -turnCurve(Math.abs(joystick.getRightX())) * 
                        inputLessThanDeadband(joystick.getRightX(), 0.03) * 
                        MaxAngularRate
                    ) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.rightBumper().whileTrue(
        //         new AlignToTagPoseHelp(true, drive, MaxSpeed, MaxAngularRate, joystick));
        // joystick.rightBumper().whileTrue(
        //     new AlignToTagPathplanner(true, drive, MaxSpeed, MaxAngularRate, joystick));
        joystick.rightBumper().whileTrue(
            new AlignToTagPose(drive, MaxSpeed, MaxAngularRate));

        joystick.leftBumper().whileTrue(
            m_drivetrain.applyRequest(() ->
            drive
                .withVelocityX(
                    -driveCurve(Math.abs(joystick.getLeftY())) * 
                        inputLessThanDeadband(joystick.getLeftY(), 0.03) * 
                        (MaxSpeed / 2.0)
                ) // Drive forward with negative Y (forward)
                .withVelocityY(
                    -driveCurve(Math.abs(joystick.getLeftX())) * 
                    inputLessThanDeadband(joystick.getLeftX(), 0.03) * 
                    (MaxSpeed / 2.0)
                ) // Drive left with negative X (left)
                .withRotationalRate(
                    -turnCurve(Math.abs(joystick.getRightX())) * 
                    inputLessThanDeadband(joystick.getRightX(), 0.03) * 
                    MaxAngularRate
                ) // Drive counterclockwise with negative X (left)
        ));

        // joystick.a().whileTrue(
        //     new AlignToAngle()
        // );

        m_elevator.setDefaultCommand(
            new RunCommand(() -> {
                m_elevator.seekPosition();
            }, m_elevator)
        );
        m_hand.setDefaultCommand(
            new RunCommand(() -> {
                m_hand.seekPosition();
            }, m_hand)
        );
        coDriverController2.axisLessThan(2, -.5).onTrue(
            new InstantCommand(() -> {photonvision.targetingLeftReef = false; System.out.println(photonvision.targetingLeftReef);}) 
        );
        coDriverController2.button(8).onTrue(
            new InstantCommand(() -> {photonvision.targetingLeftReef = true; System.out.println(photonvision.targetingLeftReef);}) 
        );

        // joystick.a().onTrue(
        //     new MoveElevatorToPose(new ElevatorPose(1.0, .75 * Math.PI), elevator, hand)
        // );
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        joystick.b().toggleOnTrue(
            new Intake()
        ).toggleOnFalse(
            new InstantCommand(() -> {
                m_hand.stop();
            })
        );
        // .toggleOnFalse(
        //     new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Stow, elevator, hand)
        // );
        // joystick.y().onTrue(
        //     new InstantCommand(() -> hand.intake())
        // ).onFalse(
        //     new InstantCommand(() -> hand.stop())
        // );

        // joystick.x().whileTrue(
        //     new SequentialCommandGroup(
        //         new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L3Score), // TODO! we don't want to drive w/ up but it's fine for now
        //         // new AlignToTagPose(true, drive, MaxSpeed, MaxAngularRate),
        //         new InstantCommand(() -> m_hand.score())
        //     )
        // );

        // joystick.a().onTrue(
        //     new InstantCommand(() -> hand.score())
        // ).onFalse(
        //     new InstantCommand(() -> hand.stop())
        // );
        // .toggleOnTrue(
        //     new InstantCommand(() -> hand.setWristSetpoint(.4))
        // ).toggleOnFalse(
        //     new InstantCommand(() -> hand.setWristSetpoint(0.0))
        // );

        coDriverController2.povDown().whileTrue(
            new RunCommand(() -> { m_hand.score(); }
        )).onFalse(
            new InstantCommand(() -> { m_hand.stop(); }
        ));

        coDriverController2.povUp().whileTrue(
            new RunCommand(() -> { m_hand.intake(); }
        )).onFalse(
            new InstantCommand(() -> { m_hand.stop(); }
        ));

        coDriverController1.button(4).onTrue( // '0'
            new InstantCommand(() -> { 
                m_coDriverControl.sendInput(CoDriverInput.L0, true);
            }
        ));

        coDriverController1.povDown().onTrue( // '1'
        new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L1, true);}
        )).onFalse( // '4'
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L1, false);}
        ));

        coDriverController1.button(1).onTrue( // '2'
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L2, true);}
        )).onFalse( // '4'
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L2, false);}
        ));

        coDriverController1.button(2).onTrue( // '3'
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L3, true);}
        )).onFalse( // '4'
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L3, false);}
        ));

        coDriverController1.button(3).onTrue( // '4'
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L4, true);}
        )).onFalse( // '4'
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L4, false);}
        ));

        coDriverController1.axisLessThan(2, -.9).whileTrue( // ')' (on third axis)
        new RunCommand(() -> {
            m_hand.setWristSetpoint(m_hand.getWristSetpoint() - .01);
        }));

        coDriverController1.axisGreaterThan(2, .9).whileTrue( // '(' (on third axis)
        new RunCommand(() -> {
            m_hand.setWristSetpoint(m_hand.getWristSetpoint() + .01);
        }));

        coDriverController1.button(8).whileTrue( // pg dn
            new RunCommand(() -> {
                m_elevator.setElevatorToPosition(m_elevator.targetPosition + 0.05);
        }));

        coDriverController1.button(7).whileTrue( // pg up
            new RunCommand(() -> {
                m_elevator.setElevatorToPosition(m_elevator.targetPosition - 0.05);
        }));

        // coDriverController1.button(8).onTrue( // pg dn
        //     new InstantCommand(() -> {
        //         elevator.setElevatorToPosition(elevator.targetPosition + 1.0);
        // }));

        coDriverController1.button(10).onTrue( // HOME
            new InstantCommand(() -> {
                m_climber.climb();
        })).onFalse(
            new InstantCommand(() -> {
                m_climber.stop();
        }));
        coDriverController1.button(11).onTrue( // S
            new InstantCommand(() -> {
                m_climber.unclimb();
        })).onFalse(
            new InstantCommand(() -> {
                m_climber.stop();
        }));

        // coDriverController1.button(8).onTrue( // TODO!
        //     new InstantCommand(() -> {
        //         m_climber.unclimb();
        // })).onFalse(
        //     new InstantCommand(() -> {
        //         m_climber.stop();
        // }));

        coDriverController2.button(6).onTrue( // Hippo
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L2Algae, true);}
        )).onFalse(
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L2Algae, false);}
        ));
        coDriverController2.button(1).onTrue( // Kebab
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L3Algae, true);}
        )).onFalse(
            new InstantCommand(() -> { m_coDriverControl.sendInput(CoDriverInput.L3Algae, false);}
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(m_drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(m_drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.button(8).onTrue(m_drivetrain.runOnce(() -> m_drivetrain.seedFieldCentric()));

        m_drivetrain.registerTelemetry(m_logger::telemeterizeDrive);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
