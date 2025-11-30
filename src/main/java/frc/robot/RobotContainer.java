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

        ChassisSpeeds driverNudgeUp = new ChassisSpeeds(0.25, 0.25, 0.25);
        driverController.povUp().whileTrue(
            Commands.run(() -> {
                swerveSubsystem.drive(driverNudgeUp);
            })
        );
    }
}
