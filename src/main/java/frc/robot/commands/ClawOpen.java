// ROBOTBUILDER TYPE: PIDCommand.

package frc.robot.commands;

import java.util.function.DoubleSupplier;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.subsystems.*;

/** A command that will open the claw */
public class ClawOpen extends PIDCommand {
/**
 * opens the claw to a specified distance/width
 * Absolutely going to need a motion path/ profiled PID command
 */
    public ClawOpen(double targetAngleDegrees, Claw ClawMotor ) {   
      super(
        new PIDController(ClawConstants.kTurnP, ClawConstants.kTurnI, ClawConstants.kTurnD),
        //Close loop
        ClawMotor::getDistance,
        //Set reference to target
        targetAngleDegrees,
        //Output pipeline to open  
        output -> ClawMotor.openClaw(output));

 //Ensure robot is stationary before it is considered to have reached the target
 getController()
        .setTolerance(ClawConstants.kTurnToleranceDeg, ClawConstants.kTurnRateToleranceDegPerS);
 }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
