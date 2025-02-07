// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.lang.management.OperatingSystemMXBean;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.elevator.KeepElevatorPosition;
import frc.robot.commands.elevator.MaintainElevatorLevel;
import frc.robot.commands.elevator.MaintainElevatorVoltage;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.commands.elevator.SetElevatorVoltage;
import frc.robot.commands.manipulator.Intake;
import frc.robot.commands.manipulator.Shoot;
import frc.robot.generated.DrivetrainConfigs;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class RobotContainer {
    private double MaxSpeed = DrivetrainConfigs.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    private final CommandXboxController operatorJoystick = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = DrivetrainConfigs.createDrivetrain();

    public final Elevator elevator = new Elevator();
    public final Manipulator manipulator = new Manipulator();
    
    private SlewRateLimiter angle_Limiter;
    private SlewRateLimiter x_Limiter;
    private SlewRateLimiter y_Limiter;

    /* Path follower */
    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
        
        angle_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_angle);
        x_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_xy);
        y_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_xy);

        configureBindings();
    }

    private void configureBindings() {
        SignalLogger.setPath("/media/sda1/ctre-logs/");
        //////////////////////////////////////// Manipulator Controls (Operator Joystick) //////////////////////////////////////////////
        operatorJoystick.leftTrigger().onTrue(new Intake(manipulator) );
        operatorJoystick.rightTrigger().onTrue(new Shoot(manipulator) );
        
        //////////////////////////////////////// Elevator Controls (Operator Joystick) //////////////////////////////////////////////
        switch(Constants.Elevator.elevatorMode) {
            case 1: 
                // 1 - Debugging with Set Speed Mode, controls elevator through directly mapping the joystick
                //     to the speed of the elevator motors
                elevator.setDefaultCommand(new ManualElevator(elevator, () -> operatorJoystick.getLeftY()));
                break;
            case 2:
                // 2 - Debugging with Set Voltage Mode, controls the elevator through always sending a 
                //     constant voltage. This voltage can be changed as the robot is enabled through buttons
                elevator.setDefaultCommand(new MaintainElevatorVoltage(elevator));
                operatorJoystick.a().onTrue(new SetElevatorVoltage(elevator, 0));
                operatorJoystick.b().onTrue(new SetElevatorVoltage(elevator, 1));
                operatorJoystick.x().onTrue(new SetElevatorVoltage(elevator, 2));
                break;
            case 3:
                // 3 - SysID Mode, allows for the running of SysID tests, will hold the elevator in the last position in between tests
                elevator.setDefaultCommand(new KeepElevatorPosition(elevator));
                operatorJoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
                operatorJoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

                operatorJoystick.back().and(operatorJoystick.y()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
                operatorJoystick.back().and(operatorJoystick.x()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
                operatorJoystick.start().and(operatorJoystick.y()).whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
                operatorJoystick.start().and(operatorJoystick.x()).whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
                break;
            default:
                // Default - Standard Mode, control elevator through buttons corresponding to L1, L2, L3, and L4, defaulting to a safe position for the elevator
                elevator.setDefaultCommand(new MaintainElevatorLevel(elevator));
                operatorJoystick.a().onTrue(new SetElevatorLevel(elevator, 1));
                operatorJoystick.b().onTrue(new SetElevatorLevel(elevator, 2));
                operatorJoystick.x().onTrue(new SetElevatorLevel(elevator, 3));
                operatorJoystick.y().onTrue(new SetElevatorLevel(elevator, 4));
                break;
        }
        


        ////////////////////////////////////////// Swerve Controls (Driver Joystick) ////////////////////////////////////////////////
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.pov(0).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.pov(180).whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        joystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        
        // reset the field-centric heading on left bumper press
        joystick.a().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        return autoChooser.getSelected();
    }
}
