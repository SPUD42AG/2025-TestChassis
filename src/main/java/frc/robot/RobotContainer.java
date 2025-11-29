package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

    @SuppressWarnings("unused")
    private final CommandXboxController driverController = new CommandXboxController(0);
    
    @SuppressWarnings("unused")
    private final CommandXboxController operatorController = new CommandXboxController(0);

    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {}
}
