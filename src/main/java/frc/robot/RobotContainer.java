package frc.robot;

import static frc.robot.Constants.SwerveConstants.*;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.SwerveSubsystem;

public class RobotContainer {
    public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
    private final CommandXboxController driverController = new CommandXboxController(DRIVE_CONTROLLER_PORT);
    public DriveCommand driveCommand = new DriveCommand(driverController, swerveSubsystem);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        swerveSubsystem.setDefaultCommand(driveCommand);

        ChassisSpeeds driverNudgeUp = new ChassisSpeeds(0.25, 0, 0);
        ChassisSpeeds driverNudgeDown = new ChassisSpeeds(-0.25, 0, 0);
        ChassisSpeeds driverNudgeLeft = new ChassisSpeeds(0, 0.25, 0);
        ChassisSpeeds driverNudgeRight = new ChassisSpeeds(0, -0.25, 0);

        driverController.povUp().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeUp);
            })
        );

        driverController.povDown().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeDown);
            })
        );

        driverController.povLeft().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeLeft);
            })
        );

        driverController.povRight().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeRight);
            })
        );

        driverController.b().whileTrue(
            Commands.run(() -> {
                IS_FIELD_RELATIVE = !IS_FIELD_RELATIVE;
            })
        );
    }
}