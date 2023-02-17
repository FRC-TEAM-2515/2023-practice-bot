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

/** A command that will rotate the turret along the x axis */
public class TurretHorizontal extends PIDCommand {
/**
 * Rotates the   to a specified angle along the y axis
 * Absolutely going to need a motion path/ profiled PID command
 */
    public TurretHorizontal(double targetAngleDegrees, Turret turretMotor ) {   
      super(
        new PIDController(TurretConstants.kTurnP, TurretConstants.kTurnI, TurretConstants.kTurnD),
        //Close loop
        turretMotor::getAngle,
        //Set reference to target
        targetAngleDegrees,
        //Output pipeline to rotate  
        output -> turretMotor.rotateTurret(output));

 //Ensure robot is stationary before it is considered to have reached the target
 getController()
        .setTolerance(TurretConstants.kTurnToleranceDeg, TurretConstants.kTurnRateToleranceDegPerS);
 }

    @Override
    public boolean isFinished() {
      // End when the controller is at the reference.
      return getController().atSetpoint();
    }
}
