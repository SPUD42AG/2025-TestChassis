package frc.robot.subsystems;

import java.io.File;
import java.io.IOException;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static frc.robot.Constants.SwerveConstants.*;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class SwerveSubsystem extends SubsystemBase {
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

    public void drive(ChassisSpeeds chassisSpeeds) {
        swerveDrive.drive(chassisSpeeds);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    static private double applyResponseCurve(double x) {
        return Math.signum(x) * Math.pow(x, 2);
    }

    public static ChassisSpeeds rotateLinearChassisSpeeds(ChassisSpeeds in, Rotation2d offset){
        Translation2d modifiedLinear = new Translation2d(
            in.vxMetersPerSecond,
            in.vyMetersPerSecond
        ).rotateBy(offset);

        return new ChassisSpeeds(
            modifiedLinear.getX(),
            modifiedLinear.getY(), 
            in.omegaRadiansPerSecond
        );
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, boolean isFieldRelative, SwerveSubsystem swerve) {
        return () -> {
            ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    
            final double inputxraw = driverController.getLeftY() * -1.0;
            final double inputyraw = driverController.getLeftX() * -1.0;
            final double inputomegaraw;
            
            inputomegaraw = driverController.getRightX() * -1.0;
    
            final double inputx = applyResponseCurve(MathUtil.applyDeadband(inputxraw, STICK_DEADBAND));
            final double inputy = applyResponseCurve(MathUtil.applyDeadband(inputyraw, STICK_DEADBAND));
            final double inputomega = applyResponseCurve(MathUtil.applyDeadband(inputomegaraw, STICK_DEADBAND));
    
            chassisSpeeds.vxMetersPerSecond = inputx * MAX_SPEED;
            chassisSpeeds.vyMetersPerSecond = inputy * MAX_SPEED;
            chassisSpeeds.omegaRadiansPerSecond = inputomega * MAX_ANGULAR_SPEED.in(RadiansPerSecond);

            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(chassisSpeeds, swerve.getPose().getRotation());
            chassisSpeeds = rotateLinearChassisSpeeds(chassisSpeeds, TELEOP_HEADING_OFFSET);
    
            return chassisSpeeds;
        };
    }

    public static Supplier<ChassisSpeeds> computeVelocitiesFromController(XboxController driverController, SwerveSubsystem swerve) {
        return computeVelocitiesFromController(driverController, IS_FIELD_RELATIVE, swerve);
    }

    public static Supplier<ChassisSpeeds> getSwerveTeleopCSSupplier(XboxController driverController, SwerveSubsystem swerve){
        return () -> {
            ChassisSpeeds chassisSpeeds = computeVelocitiesFromController(driverController, swerve).get();
            return chassisSpeeds;
        };
    }
}
