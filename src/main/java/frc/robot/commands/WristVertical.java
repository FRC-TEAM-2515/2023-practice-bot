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

/** A command that will rotate the wrist along the y axis */
public class WristVertical extends PIDCommand {
/**
 * Rotates the wrist to a specified angle along the y axis
 * Absolutely going to need a motion path/ profiled PID command
 */
    public WristVertical(double targetAngleDegrees, Wrist verticalMotorWrist) {   
      super(
        new PIDController(VerticalWristConstants.kTurnP, VerticalWristConstants.kTurnI, VerticalWristConstants.kTurnD),
        //Close loop
        verticalMotorWrist::getAngle,
        //Set reference to target
        targetAngleDegrees,
        //Output pipeline to rotate wrist
        output -> verticalMotorWrist.rotateWristVertical(output));

 //Ensure robot is stationary before it is considered to have reached the target
 getController()
        .setTolerance(VerticalWristConstants.kTurnToleranceDeg, VerticalWristConstants.kTurnRateToleranceDegPerS);
 }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
