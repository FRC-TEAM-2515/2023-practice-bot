package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.RobotContainer;

/**
 * Safely resets the robot.
 */
public class SafeReset extends InstantCommand {

    private RobotContainer robotContainer;
    // Called once when this command runs
    @Override
    public void initialize() {
        robotContainer.safeReset();
    }

    @Override
    public boolean runsWhenDisabled() {
        return false;

    }
}
