package frc.robot;

import edu.wpi.first.net.WebServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    @SuppressWarnings("unused")
    private final RobotContainer robotContainer;

    public Robot() {
        robotContainer = new RobotContainer();
    }

    @Override
    public void robotInit() {
        WebServer.start(5800, Filesystem.getDeployDirectory().getPath());
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {}

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        DriverStation.silenceJoystickConnectionWarning(true);
    }

    @Override
    public void simulationPeriodic() {}
}
