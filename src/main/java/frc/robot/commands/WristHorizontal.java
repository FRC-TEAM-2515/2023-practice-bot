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

/** A command that will rotate the wrist along the x axis */
public class WristHorizontal extends PIDCommand {
/**
 * Rotates the wrist to a specified angle along the x axis
 * Absolutely going to need a motion path/ profiled PID command
 */
    public WristHorizontal(double targetAngleDegrees, Wrist horizontalMotorWrist) {   
      super(
        new PIDController(HorizontalWristConstants.kTurnP, HorizontalWristConstants.kTurnI, HorizontalWristConstants.kTurnD),
        //Close loop
        horizontalMotorWrist::getAngle,
        //Set reference to target
        targetAngleDegrees,
        //Output pipeline to rotate wrist
        output -> horizontalMotorWrist.rotateWristHorizontal(output));

 //Ensure robot is stationary before it is considered to have reached the target
 getController()
        .setTolerance(HorizontalWristConstants.kTurnToleranceDeg, HorizontalWristConstants.kTurnRateToleranceDegPerS);
 }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
