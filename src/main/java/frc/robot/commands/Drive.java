package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import static frc.robot.subsystems.SwerveSubsystem.*;
import frc.robot.subsystems.SwerveSubsystem;

public class Drive extends Command {
    private final SwerveSubsystem swerveSubsystem;
    private final CommandXboxController driverController;

    public Drive(CommandXboxController driverController, SwerveSubsystem swerveSubsystem) {
        this.swerveSubsystem = swerveSubsystem;
        this.driverController = driverController;

        addRequirements(swerveSubsystem);
    }

    @Override
    public void execute() {
        ChassisSpeeds chassisSpeeds = getSwerveTeleopCSSupplier(driverController.getHID(), swerveSubsystem).get();
        swerveSubsystem.drive(chassisSpeeds);
    }
}
