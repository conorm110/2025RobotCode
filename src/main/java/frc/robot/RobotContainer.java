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
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.elevator.KeepElevatorPosition;
import frc.robot.commands.elevator.MaintainElevatorLevel;
import frc.robot.commands.elevator.MaintainElevatorVoltage;
import frc.robot.commands.elevator.ManualElevator;
import frc.robot.commands.elevator.SetElevatorLevel;
import frc.robot.commands.elevator.SetElevatorVoltage;
import frc.robot.commands.manipulator.ActivateAlgaeArm;
import frc.robot.commands.manipulator.BackCoralToSensor;
import frc.robot.commands.manipulator.DeactivateAlgaeArm;
import frc.robot.commands.manipulator.Intake;
import frc.robot.commands.manipulator.Shoot;
import frc.robot.commands.manipulator.ShooterReverse;
import frc.robot.generated.DrivetrainConfigs;
import frc.robot.subsystems.AlgaeArm;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Manipulator;

public class RobotContainer {
    /* Joystick Initializations */
    private final CommandXboxController joystick = new CommandXboxController(0);
    private SlewRateLimiter angle_Limiter; // limiter for driver rotation
    private SlewRateLimiter x_Limiter; // limiter for Driver X speed
    private SlewRateLimiter y_Limiter; // limiter for Driver y speed
    private final CommandXboxController operatorJoystick = new CommandXboxController(1);
    private Trigger leftTrigger; // Trigger event for operator left trigger
    private Trigger rightTrigger; // Trigger event for operator right trigger
    private Trigger elevavorIsHighTrigger;    
    /* Command Drivetrain Initializations */
    public final CommandSwerveDrivetrain drivetrain = DrivetrainConfigs.createDrivetrain();

    public double MaxSpeed = DrivetrainConfigs.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top
                                                                                     // speed
    private final Telemetry logger = new Telemetry(MaxSpeed); // Logger for SysId - not used when SysId commented out
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second
                                                                                      // max angular velocity

    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake(); // Brake Mode
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric() // Field Centric Mode
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric() // Robot Centric Mode
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    private final ProfiledFieldCentricFacingAngle drivetrainTargetAngle = // Field Centric Target Angle Mode
            new ProfiledFieldCentricFacingAngle(new TrapezoidProfile.Constraints(MaxAngularRate, MaxAngularRate / 0.25))
                    .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
                    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    /* Other subsystem Initializations */
    public final Elevator elevator = new Elevator();
    public final Manipulator manipulator = new Manipulator();
    private AlgaeArm algaeArm = new AlgaeArm();

    /* Autonomous Command Declaration */
    private final SendableChooser<Command> autoChooser;


    public RobotContainer() {

        // Limiters for Driver Joystick
        angle_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_angle);
        x_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_xy);
        y_Limiter = new SlewRateLimiter(Constants.Swerve.joystickSlewLimiter_xy);

        // Trigger Events for Operator Left/Right Trigger Presses
        leftTrigger = new Trigger(() -> operatorJoystick.getLeftTriggerAxis() > 0.2);
        rightTrigger = new Trigger(() -> operatorJoystick.getRightTriggerAxis() > 0.2);
        elevavorIsHighTrigger = new Trigger(() -> elevator.getLevel() > 2);

        // Configure Bindings
        configureBindings();

        // Named Commands (Needed for Path Planner Autos)
        NamedCommands.registerCommand("Shoot", new Shoot(manipulator));
        NamedCommands.registerCommand("Intake", new Intake(manipulator));
        NamedCommands.registerCommand("BackUpCoral", new BackCoralToSensor(manipulator));
        NamedCommands.registerCommand("Remove Algae", new ActivateAlgaeArm(algaeArm, elevator));
        NamedCommands.registerCommand("L1", new SetElevatorLevel(elevator, algaeArm, 1));
        NamedCommands.registerCommand("L2", new SetElevatorLevel(elevator, algaeArm, 2));
        NamedCommands.registerCommand("L3", new SetElevatorLevel(elevator, algaeArm, 3));
        NamedCommands.registerCommand("L4", new SetElevatorLevel(elevator, algaeArm, 4));
        NamedCommands.registerCommand("Maintain Level", new MaintainElevatorLevel(elevator));

        // Read Selected Autonomous
        autoChooser = AutoBuilder.buildAutoChooser("New Auto");
        SmartDashboard.putData("Auto Mode", autoChooser);
    }

    private void configureBindings() {
        /************************************************ Operator Controls **************************************************/

        /** 
         * Manipulator Controls
         **/
        // IMPORTANT: Driver Right Trigger Will Also Shoot, Rest Of Controls For Operator Only
        // Left Bumper   - Intake
        // Right Bumper  - Shoot
        // Left Trigger  - Put Away Algae Arm
        // Right Trigger - Activate Algae Arm
        // Start Button  - Reverse Shooter Motors 
        operatorJoystick.leftBumper().onTrue(new Intake(manipulator).andThen(new BackCoralToSensor(manipulator)));
        operatorJoystick.rightBumper().or(joystick.rightTrigger()).onTrue(new Shoot(manipulator));
        operatorJoystick.rightTrigger().onTrue(new ActivateAlgaeArm(algaeArm, elevator));
        operatorJoystick.leftTrigger().onTrue(new DeactivateAlgaeArm(algaeArm));
        operatorJoystick.start().whileTrue(new ShooterReverse(manipulator)); // bindings interfere with elevator SysID bindings, normally not a problem

        /** 
         * Elevator Controls
         **/
        // Default Mode (when Constants.Elevator.elevatorMode == 0, normal setpoint operation)
        // Press A - Set Elevator to L1
        // Press B - Set Elevator to L2
        // Press X - Set Elevator to L3
        // Press Y - Set Elevator to L4
        configureElevatorBindings(Constants.Elevator.elevatorMode); // See Method or Constants Def. for Info On Other Modes


        /************************************************ Driver Controls **************************************************/

        joystick.start().and(new Trigger(() -> MaxSpeed > 0)).onTrue(new InstantCommand(drivetrain::resetFieldOriented, drivetrain));
        joystick.back().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.start().and(new Trigger(() -> MaxSpeed < 0)).whileTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        /**
         * Field Centric Driving Mode                       -- Applied By Default
         **/
        drivetrain.setDefaultCommand(
                drivetrain.applyRequest(() -> drive.withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)
                        .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate)));

        /** 
         * Field Oriented w/ Constant Angle Driving Mode    -- Applied while following button held
         **/
        // A - Shooter Facing Driver's Side Middle Coral Reef
        // A and X - Shooter Facing Driver's Side Left Coral Reef
        // A and B - Shooter Facing Driver's Side Right Coral Reef
        // Y - Shooter Facing Barge Side Middle Coral Reef
        // Y and X - Shooter Facing Barge Side Left Coral Reef
        // Y and B - Shooter Facing Barge Side Right Coral Reef
        // X - Hopper Facing Left Intake Station
        // Y - Hopper Facing Right Intake Station
        configureFieldOrientedWithConstantAngleBindings();

        /**
         * Slow Field Centric Driving Mode                  -- Applied while Right Bumper Held
         */
        joystick.rightBumper().or(elevavorIsHighTrigger).whileTrue(
                drivetrain.applyRequest(() -> drive
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed * 0.3)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed * 0.3)
                        .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate * 0.3)));

        /**
         * Slow Robot Centric Driving Mode                  -- Applied while Left Bumper Held
         */
        joystick.leftBumper().whileTrue(
                drivetrain.applyRequest(() -> driveRobotCentric
                        .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed * 0.3)
                        .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed * 0.3)
                        .withRotationalRate(-angle_Limiter.calculate(joystick.getRightX()) * MaxAngularRate * 0.3)));

        /**
         * Sys ID Button Mappings For Drivetrain
         **/
        // Keep commented out when not in use
        // SignalLogger.setPath("/media/sda1/ctre-logs/"); IMPORTANT: MUST BE UNCOMMENTED FOR TESTING ELEVATOR SYSID TOO!!
        // configureSysIdBindings();
    }

    /** 
     * getAutonomousCommand - Run the Path Selected from the Auto Chooser
     **/
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    /**
     * configureSysIdBindings() - Configures SysID bindings for the drivetrain. Keep
     *                            commented out when not in use
     */
    public void configureSysIdBindings() {
        joystick.leftBumper().and(joystick.start()).onTrue(Commands.runOnce(SignalLogger::start));
        joystick.rightBumper().and(joystick.start()).onTrue(Commands.runOnce(SignalLogger::stop));
        drivetrain.registerTelemetry(logger::telemeterize);
        joystick.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)); // First
        joystick.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); // Second
        joystick.povLeft().whileTrue(drivetrain.sysIdDynamic(Direction.kForward)); // Third
        joystick.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse)); // Fourth
    }

    /**
     * configureFieldOrientedWithConstantAngleBindings() - Hold the following button
     * combinations while driving with the normal driver joysticks to drive with 
     * selected orientation:
     * 
     * A - Shooter Facing Driver's Side Middle Coral Reef
     * A and X - Shooter Facing Driver's Side Left Coral Reef
     * A and B - Shooter Facing Driver's Side Right Coral Reef
     * Y - Shooter Facing Barge Side Middle Coral Reef
     * Y and X - Shooter Facing Barge Side Left Coral Reef
     * Y and B - Shooter Facing Barge Side Right Coral Reef
     * X - Hopper Facing Left Intake Station
     * Y - Hopper Facing Right Intake Station
     * 
     **/
    private void configureFieldOrientedWithConstantAngleBindings() {
        joystick.x().and(joystick.a().debounce(0.15)).whileTrue(
                drivetrain.applyRequest(
                        () -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_LeftCloseReef)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));
        joystick.a().whileTrue(
                drivetrain.applyRequest(
                        () -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_MiddleCloseReef)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));
        joystick.a().and(joystick.b().debounce(0.15)).whileTrue(
                drivetrain.applyRequest(
                        () -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_RightCloseReef)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));
        joystick.y().and(joystick.x()).debounce(0.15).whileTrue(
                drivetrain
                        .applyRequest(() -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_LeftFarReef)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));
        joystick.y().whileTrue(
                drivetrain.applyRequest(
                        () -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_MiddleFarReef)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));

        joystick.y().and(joystick.b().debounce(0.15)).whileTrue(
                drivetrain.applyRequest(
                        () -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_RightFarReef)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));
        
        joystick.x().whileTrue(
                drivetrain.applyRequest(
                        () -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_LeftCoralStation)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));

        joystick.b().whileTrue(
                drivetrain.applyRequest(
                        () -> drivetrainTargetAngle.withTargetDirection(Constants.Field.angle_RightCoralStation)
                                .withVelocityX(-y_Limiter.calculate(joystick.getLeftY()) * MaxSpeed)
                                .withVelocityY(-x_Limiter.calculate(joystick.getLeftX()) * MaxSpeed)));
    }

    /**
     * configureElevatorBindings() - Depending on the provided parameter, configure
     * operator joystick button bindings for the elevator:
     * 
     * 0 [ Default ] - Standard mode, set the elevator to hold at L1, L2, L3, or 
     *                 L4 by pressing A, B, X, or Y respectivly 
     *             1 - Debugging with Set Speed Mode, controls elevator through 
     *                 directly mapping the joystick to the speed of the elevator
     *                 motors
     *             2 - Debugging with set voltage mode, controls the elevator
     *                 through always sending a constant voltage. This voltage
     *                 can be changed as the robot is enabled through btn presses
     *             3 - SysID Mode, allows for the running of SysID tests, will 
     *                 hold the elevator in the last position in between tests
     */
    private void configureElevatorBindings(int elevatorMode) {
        switch (elevatorMode) {
                case 1:
                    elevator.setDefaultCommand(new ManualElevator(elevator, () -> -operatorJoystick.getLeftY()));
                    break;
                case 2:
                    elevator.setDefaultCommand(new MaintainElevatorVoltage(elevator));
                    operatorJoystick.a().onTrue(new SetElevatorVoltage(elevator, 0));
                    operatorJoystick.b().onTrue(new SetElevatorVoltage(elevator, 1));
                    operatorJoystick.x().onTrue(new SetElevatorVoltage(elevator, 2));
                    break;
                case 3:
                    elevator.setDefaultCommand(new KeepElevatorPosition(elevator));
                    operatorJoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));
                    operatorJoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    
                    operatorJoystick.back().and(operatorJoystick.y()).whileTrue(elevator.sysIdDynamic(Direction.kForward));
                    operatorJoystick.back().and(operatorJoystick.x()).whileTrue(elevator.sysIdDynamic(Direction.kReverse));
                    operatorJoystick.start().and(operatorJoystick.y())
                            .whileTrue(elevator.sysIdQuasistatic(Direction.kForward));
                    operatorJoystick.start().and(operatorJoystick.x())
                            .whileTrue(elevator.sysIdQuasistatic(Direction.kReverse));
                    break;
                default:
                    elevator.setDefaultCommand(new MaintainElevatorLevel(elevator));
                    operatorJoystick.a().onTrue(new SetElevatorLevel(elevator, algaeArm, 1));
                    operatorJoystick.b().onTrue(new SetElevatorLevel(elevator, algaeArm, 2));
                    operatorJoystick.x().onTrue(new SetElevatorLevel(elevator, algaeArm, 3));
                    operatorJoystick.y().onTrue(new SetElevatorLevel(elevator, algaeArm, 4));
                    operatorJoystick.back()
                            .whileTrue(new ManualElevator(elevator, () -> (operatorJoystick.getRightY() * -0.25)));
                    break;
            }
    }
}
