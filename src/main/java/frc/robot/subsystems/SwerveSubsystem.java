package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Filesystem;

import static edu.wpi.first.units.Units.Meter;
import static frc.robot.Constants.SwerveConstants.*;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem {
    @SuppressWarnings("unused")
    private final SwerveDrive swerveDrive;
    private final File directory = new File(Filesystem.getDeployDirectory(),"swerve");

    public SwerveSubsystem() {
        try {
            swerveDrive = new SwerveParser(directory).createSwerveDrive(
                MAX_SPEED,
                new Pose2d(new Translation2d(Meter.of(2),
                Meter.of(5)),
                Rotation2d.fromDegrees(180))
            );
        } catch (IOException e) {
            throw new RuntimeException(e);
        }
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH;
    }
}
