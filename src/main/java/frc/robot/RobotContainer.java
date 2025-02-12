// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.commands.AlignToTagPose;
import frc.robot.commands.Intake;
import frc.robot.commands.MoveElevatorToPoseAndScore;
import frc.robot.commands.MoveElevatorToPoseBezier;
import frc.robot.constants.ElevatorCommandConstants;
import frc.robot.constants.ElevatorCommandConstants.ElevatorPose;
import frc.robot.generated.TunerConstants;
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
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController coDriverController1 = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Elevator elevator = new Elevator();
    public final Hand hand = new Hand();

    public final PhotonVision photonvision = new PhotonVision(drivetrain);

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        
        autoChooser = AutoBuilder.buildAutoChooser("Tests");
        
        SmartDashboard.putData("Auto Mode", autoChooser);

        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive
                    .withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), 0.03) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), 0.03) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-MathUtil.applyDeadband(joystick.getRightX(), 0.03) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        elevator.setDefaultCommand(
            new RunCommand(() -> {
                elevator.seekPosition();
            }, elevator)
        );
        hand.setDefaultCommand(
            new RunCommand(() -> {
                hand.seekPosition();
            }, hand)
        );

        // joystick.a().onTrue(
        //     new MoveElevatorToPose(new ElevatorPose(1.0, .75 * Math.PI), elevator, hand)
        // );
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        joystick.b().toggleOnTrue(
            new Intake(elevator, hand)
        );
        // .toggleOnFalse(
        //     new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Stow, elevator, hand)
        // );
        // joystick.y().whileTrue(
        //     new AlignToTagPose(drivetrain, drive, photonvision, MaxAngularRate, MaxAngularRate)
        // );
        joystick.y().onTrue(
            new InstantCommand(() -> hand.intake())
        ).onFalse(
            new InstantCommand(() -> hand.stop())
        );

        joystick.x().toggleOnTrue(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L4Score, elevator, hand)
        ).toggleOnFalse(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Stow, elevator, hand)
        );

        joystick.a().onTrue(
            new InstantCommand(() -> hand.score())
        ).onFalse(
            new InstantCommand(() -> hand.stop())
        );
        // .toggleOnTrue(
        //     new InstantCommand(() -> hand.setWristSetpoint(.4))
        // ).toggleOnFalse(
        //     new InstantCommand(() -> hand.setWristSetpoint(0.0))
        // );

        coDriverController1.povDown().toggleOnTrue(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L1Score, elevator, hand)
        ).toggleOnFalse(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Stow, elevator, hand)
        );

        coDriverController1.button(1).toggleOnTrue(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L2Score, elevator, hand)
        ).toggleOnFalse(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Stow, elevator, hand)
        );
        coDriverController1.button(2).toggleOnTrue(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L3Score, elevator, hand)
        ).toggleOnFalse(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Stow, elevator, hand)
        );
        coDriverController1.button(3).toggleOnTrue(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.L4Score, elevator, hand)
        ).toggleOnFalse(
            new MoveElevatorToPoseAndScore(ElevatorCommandConstants.Stow, elevator, hand)
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.rightBumper().whileTrue(
            new InstantCommand(
                () -> {
                    elevator.setElevatorToPosition(elevator.targetPosition - 1.0);
                }
            )
        );
        joystick.rightTrigger().whileTrue(
            new InstantCommand(
                () -> {
                    elevator.setElevatorToPosition(elevator.targetPosition + 1.0);
                }
            )
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
