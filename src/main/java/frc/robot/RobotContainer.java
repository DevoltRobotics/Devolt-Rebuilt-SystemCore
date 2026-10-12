// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static org.wpilib.units.Units.*;

import org.wpilib.command2.button.CommandGamepad;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.limelightvision.Limelight;
import com.ctre.phoenix6.swerve.SwerveRequest;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.button.CommandXboxController;
import org.wpilib.command2.button.RobotModeTriggers;
import org.wpilib.command2.button.Trigger;
import org.wpilib.command2.sysid.SysIdRoutine.Direction;
import org.wpilib.driverstation.Gamepad.Button;

import frc.robot.Commands.SOTFCommand;
import frc.robot.Constants.PoseinField;
import frc.robot.Constants.TurretsPos;
import frc.robot.Constants.CANId.CAN_s2;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.Subsystems.TurretSubsystem;
import frc.robot.generated.TunerConstants;

public class RobotContainer {
        private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

    private final SwerveTelemetry logger = new SwerveTelemetry(MaxSpeed);

    public final CommandGamepad joystick = new CommandGamepad(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public final TransferSubsystem transferSubsystem = new TransferSubsystem();

    public final ShooterSubsystem shooterLeftSubsystem = new ShooterSubsystem(CAN_s2.LFlywheelCan, drivetrain.getState().Pose);
    public final TurretSubsystem turretLeftSubsystem = new TurretSubsystem(CAN_s2.LTurretCan, TurretsPos.LeftTurretOffset);

    public final ShooterSubsystem shooterRightSubsystem = new ShooterSubsystem(CAN_s2.RFlywheelCan, drivetrain.getState().Pose);
    //public final TurretSubsystem turretRightSubsystem = new TurretSubsystem(CAN_s2.RTurretCan);
    
    public final Limelight limelight = new Limelight("");

    public RobotContainer() {
        configureBindings();
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(transferSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterLeftSubsystem);
        CommandScheduler.getInstance().registerSubsystem(turretLeftSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterRightSubsystem);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * (1-joystick.getRawAxis(2)*.8)) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed * (1-joystick.getRawAxis(2)*.8)) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

       /*  joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));*/

        // reset the field-centric heading on left bumper press
       // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.rightBumper().onTrue(
            new ParallelCommandGroup(
                intakeSubsystem.pivotDownCMD(),    
                intakeSubsystem.IntakeinCMD(intakeSubsystem)
            )
        );

        joystick.rightBumper().onFalse(intakeSubsystem.pivotDownSafeCMD());
    
        joystick.rightBumper().onFalse(intakeSubsystem.IntakeStopCMD(intakeSubsystem));

        joystick.button(Button.DPAD_DOWN).onTrue(intakeSubsystem.pivotDownSafeCMD());
        joystick.button(Button.DPAD_UP).onTrue(intakeSubsystem.pivotUpCMD());

        joystick.leftBumper().onTrue(transferSubsystem.TransferShootCMD(transferSubsystem));
        joystick.leftBumper().onFalse(transferSubsystem.StopTransferCMD(transferSubsystem));

        joystick.rightStick().onTrue(turretLeftSubsystem.SetTurretPosCMD(0));
        joystick.leftStick().onTrue(turretLeftSubsystem.SetTurretPosCMD(89));

        joystick.button(Button.DPAD_RIGHT).onTrue(shooterLeftSubsystem.SetVelocityCMD(100));
        joystick.button(Button.DPAD_LEFT).onTrue(shooterRightSubsystem.SetVelocityCMD(17));

        joystick.button(Button.FACE_DOWN).onTrue(turretLeftSubsystem.resetOffsetCMD());
        joystick.button(Button.FACE_LEFT).onTrue(new SOTFCommand(drivetrain, shooterLeftSubsystem, turretLeftSubsystem));
        joystick.button(Button.FACE_UP).onTrue(new InstantCommand(()->drivetrain.resetRotation(new Rotation2d(0))));
    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
