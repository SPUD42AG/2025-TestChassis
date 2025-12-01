package frc.robot;

import static edu.wpi.first.units.Units.RadiansPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;

public final class Constants {
    public static final class SwerveConstants {
        public static final double TRACK_WIDTH = 22.475 / 12;
        public static final double WHEEL_BASE = 22.475 / 12;
        public static final double CHASSIS_RADIUS = Math.hypot(
                TRACK_WIDTH / 2, WHEEL_BASE / 2);

        public static final double MAX_SPEED = Units.feetToMeters(12);
        public static final AngularVelocity MAX_ANGULAR_SPEED = RadiansPerSecond.of(MAX_SPEED * Math.PI / CHASSIS_RADIUS);

        public static boolean IS_FIELD_RELATIVE = false;

        public static final double STICK_DEADBAND = 0.05;

        public static final Rotation2d TELEOP_HEADING_OFFSET = Rotation2d.fromDegrees(0.0);

        public static final int DRIVE_CONTROLLER_PORT = 0;
    }
}