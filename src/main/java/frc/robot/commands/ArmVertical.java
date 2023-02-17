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

/** A command that will rotate the Arm along the y axis */
public class ArmVertical extends PIDCommand {
/**
 * Rotates the Arm to a specified angle along the y axis
 * Absolutely going to need a motion path/ profiled PID command
 */
    public ArmVertical(double targetAngleDegrees, Arm armMotor) {   
      super(
        new PIDController(ArmConstants.kTurnP, ArmConstants.kTurnI, ArmConstants.kTurnD),
        //Close loop
        armMotor::getAngle,
        //Set reference to target
        targetAngleDegrees,
        //Output pipeline to rotate Arm
        output -> armMotor.rotateArm(output));

 //Ensure robot is stationary before it is considered to have reached the target
 getController()
        .setTolerance(ArmConstants.kTurnToleranceDeg, ArmConstants.kTurnRateToleranceDegPerS);
 }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
