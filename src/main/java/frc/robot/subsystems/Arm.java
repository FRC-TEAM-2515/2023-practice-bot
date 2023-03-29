package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.util.RobotMath;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import frc.robot.util.OIReporters;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.wpilibj.Joystick;

import javax.lang.model.util.ElementScanner14;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*; 
import java.lang.Math; 

/**
 *
 */
public class Arm extends SubsystemBase {

private CANSparkMax j1Turret; 
private CANSparkMax j2Elbow;
private CANSparkMax j3WristY;
//private CANSparkMax j4WristX;
private CANSparkMax j5Claw;

private SparkMaxAbsoluteEncoder j1TurretEncoder; 
private SparkMaxAbsoluteEncoder j2ElbowEncoder;
private SparkMaxAbsoluteEncoder j3WristYEncoder;
//private SparkMaxAbsoluteEncoder j4WristXEncoder;
private SparkMaxAbsoluteEncoder j5ClawEncoder;

// private RelativeEncoder j1TurretEncoder; 
// private SparkMaxAbsoluteEncoder j2ElbowEncoder;
// private RelativeEncoder j3WristYEncoder;
// private RelativeEncoder j4WristXEncoder;
// private RelativeEncoder j5ClawEncoder;

//PID Variables
double j1_kP = 0;
double j1_kI = 0;
double j1_kD = 0;
private double j1_desiredPosition = 0;
double j2_kP = 0;
double j2_kI = 0;
double j2_kD = 0;
private double j2_desiredPosition = 0;
double j3_kP = 0;
double j3_kI = 0;
double j3_kD = 0;
private double j3_desiredPosition = 0;
double j5_kP = 0;
double j5_kI = 0;
double j5_kD = 0;
private double j5_desiredPosition = 0;

//PID Controllers
private PIDController j1_pid = new PIDController(j1_kP, j1_kI, j1_kD);
private PIDController j2_pid = new PIDController(j2_kP, j2_kI, j2_kD);
private PIDController j3_pid = new PIDController(j3_kP, j3_kI, j3_kD);
private PIDController j5_pid = new PIDController(j5_kP, j5_kI, j5_kD);
 
private SlewRateLimiter armRateLimiter = new SlewRateLimiter(ArmConstants.armSlewRateLimiter);
private int controlMode = 0;


protected XboxController armController;

    public Arm()
    {
         
        j1Turret = new CANSparkMax(11, MotorType.kBrushless); //10:1 MAG absolute encoder
        j2Elbow = new CANSparkMax(12, MotorType.kBrushless); //210:1 MAG absolute encoder
        j3WristY = new CANSparkMax(13, MotorType.kBrushless); //5:1 MAG absolute encoder
        //j4WristX = new CANSparkMax(14, MotorType.kBrushless); //built-in relative encoder
        j5Claw = new CANSparkMax(15, MotorType.kBrushless); //10:1

        j1Turret.restoreFactoryDefaults();  
        j1Turret.setInverted(false);
        //j1Turret.setIdleMode(IdleMode.kBrake);
        j1Turret.setIdleMode(IdleMode.kCoast);
        j1Turret.burnFlash();

        j2Elbow.restoreFactoryDefaults();  
        j2Elbow.setInverted(true);
        //j2Elbow.setIdleMode(IdleMode.kBrake);
        j2Elbow.setIdleMode(IdleMode.kCoast);
        j2Elbow.burnFlash();
        
        j3WristY.restoreFactoryDefaults();  
        j3WristY.setInverted(false);
        //j3WristY.setIdleMode(IdleMode.kBrake);
        j3WristY.setIdleMode(IdleMode.kCoast);
        j3WristY.burnFlash();

        //j4WristX.restoreFactoryDefaults();  
        //j4WristX.setInverted(false);
        //j4WristX.setIdleMode(IdleMode.kBrake);
        //j4WristX.setIdleMode(IdleMode.kCoast);
        //j4WristX.burnFlash();

        j5Claw.restoreFactoryDefaults();  
        j5Claw.setInverted(false);
        //j5Claw.setIdleMode(IdleMode.kBrake);
        j5Claw.setIdleMode(IdleMode.kCoast);
        j5Claw.burnFlash();

        j1TurretEncoder =  j1Turret.getAbsoluteEncoder(Type.kDutyCycle);
        j2ElbowEncoder =  j2Elbow.getAbsoluteEncoder(Type.kDutyCycle);
        j3WristYEncoder =  j3WristY.getAbsoluteEncoder(Type.kDutyCycle);
        //j4WristXEncoder =  j4WristX.getAbsoluteEncoder(Type.kDutyCycle);
        j5ClawEncoder =  j5Claw.getAbsoluteEncoder(Type.kDutyCycle);
        

        // j1TurretEncoder = j1Turret.getEncoder();
        // j2ElbowEncoder =  j2Elbow.getAbsoluteEncoder(Type.kDutyCycle);
        // j3WristYEncoder =  j3WristY.getEncoder();
        // j4WristXEncoder =  j4WristX.getEncoder();
        // j5ClawEncoder =  j5Claw.getEncoder();        
    }

    public void manualControl()
    {
        double leftX = RobotContainer.getInstance().getOI().getArmController().getLeftX();
        double leftY = RobotContainer.getInstance().getOI().getArmController().getLeftY();
        double rightY =RobotContainer.getInstance().getOI().getArmController().getRightY();
        double rightTrigger = RobotContainer.getInstance().getOI().getArmController().getRightTriggerAxis();
        double leftTrigger = RobotContainer.getInstance().getOI().getArmController().getLeftTriggerAxis();

        boolean button_x = RobotContainer.getInstance().getOI().getArmController().getXButtonPressed();
        boolean button_y = RobotContainer.getInstance().getOI().getArmController().getYButtonPressed();
        boolean button_a = RobotContainer.getInstance().getOI().getArmController().getAButtonPressed();
        boolean button_b = RobotContainer.getInstance().getOI().getArmController().getBButtonPressed();


        moveElbowY(leftY);
        moveWristY(rightY);
        moveElbowX(leftX);
        openClaw(leftTrigger);
        closeClaw(rightTrigger);

        //used for auto-positioning of arm
        // position1(button_a);
        // position2(button_b);
        // position3(button_x);
        // position4(button_y);

    }
    

    // public CommandBase unlimitedManualControl(double turnJ1Turret, double moveJ2Elbow, double rotatej3WristY, double rotatej4WristX, double openClaw, double closeClaw) {

    //     return run(() ->
    //     this.j1Turret.set(turnJ1Turret)).alongWith(run(() ->
    //     this.j2Elbow.set(moveJ2Elbow))).alongWith(run(() ->
	// 	   this.j3WristY.set(rotatej3WristY))).alongWith(run(() ->
	// 	   this.j4WristX.set(rotatej4WristX))).alongWith(run(() ->
    //     this.j5Claw.set(openClaw))).alongWith(run(() -> 
    //     this.j5Claw.set(-closeClaw)));
    // }

    public void updateSmartDashboard()
    {

        SmartDashboard.putNumber("j1TurretEncoder", j1TurretEncoder.getPosition());
        SmartDashboard.putNumber("j2ElbowEncoder", j2ElbowEncoder.getPosition());
        SmartDashboard.putNumber("j3WristYEncoder", j3WristYEncoder.getPosition());
        //SmartDashboard.putNumber("j4WristXEncoder", j4WristXEncoder.getPosition()); 
        SmartDashboard.putNumber("j5ClawEncoder", j5ClawEncoder.getPosition());

        SmartDashboard.putNumber("j1TurretEncoder D", RobotMath.j1EncoderConvertDegrees(j1TurretEncoder.getPosition()));
        SmartDashboard.putNumber("j2ElbowEncoder D", RobotMath.j2EncoderConvertDegrees(j2ElbowEncoder.getPosition()));
        SmartDashboard.putNumber("j3WristYEncoder D", RobotMath.j3EncoderConvertDegrees(j3WristYEncoder.getPosition()));
        //SmartDashboard.putNumber("j4WristXEncoder D", RobotMath.j4EncoderConvertDegrees(j4WristXEncoder.getPosition())); 
        SmartDashboard.putNumber("j5ClawEncoder D", RobotMath.j5EncoderConvertDegrees(j5ClawEncoder.getPosition()));
    }

    public void resetArmEncoders()
    {
        //j1TurretEncoder.setPosition(0);
        //j3WristYEncoder.setPosition(0);
        //j4WristXEncoder.setPosition(0);
        //j5ClawEncoder.setPosition(0);
    }
    
    @Override
    public void periodic()
    {
        //super.periodic(); //needed if PID motion profiling subsystem
       updateSmartDashboard();
       var j1_error = j1_pid.calculate(j1TurretEncoder.getPosition());
       j1Turret.set(j1_error);
       var j2_error = j2_pid.calculate(j2ElbowEncoder.getPosition());
       j2Elbow.set(j2_error);
       var j3_error = j3_pid.calculate(j3WristYEncoder.getPosition());
       j3WristY.set(j3_error);
       var j5_error = j5_pid.calculate(j5ClawEncoder.getPosition());
       j5Claw.set(j5_error);
    
    }

    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run when in simulation

    }

    public void moveElbowX(double leftX) //moves turret left and right - assumed smallest encoder value is on right side
    {
        double currentPosition = j1TurretEncoder.getPosition();
        double maxRightValue = 0; //value of encoder at rightmost turret position
        double maxLeftValue = 1; //value of encoder at leftmost turret position
        double speedLimiter = 0.5; //max motor voltage

        if( Math.abs(leftX) < 0.2 && maxRightValue < currentPosition && currentPosition < maxLeftValue )
        {
            j1_desiredPosition += (leftX * speedLimiter);
            j1_pid.setSetpoint(j1_desiredPosition);
        }


        // if( maxLeftValue > currentPosition && currentPosition > maxRightValue ) //prevents turret from exceeding bounds
        // {
        //     if( leftX < 0 ) //rotates turret to left - assumed run motor forwards
        //     {
        //         //runs motor at speedLimiter*x^2 where x is leftX //value is between 0-speedLimiter //squared to create curve
        //         j1Turret.set(speedLimiter*Math.pow(getDeadZoneAdjustment(leftX,ArmConstants.controllerDeadzone),2));
        //         SmartDashboard.putNumber("Turret Rotation Force", speedLimiter*Math.pow(getDeadZoneAdjustment(leftX,ArmConstants.controllerDeadzone),2));
        //     }
        //     else if( leftX > 0 ) //rotates turret to right - assumed run motor backwards
        //     {
        //         //runs motor at -speedLimiter*x^2 where x is leftX //value is between -speedLimiter-0 //squared to create curve
        //         j1Turret.set(-speedLimiter*Math.pow(getDeadZoneAdjustment(leftX,ArmConstants.controllerDeadzone),2));
        //         SmartDashboard.putNumber("Turret Rotation Force", -speedLimiter*Math.pow(getDeadZoneAdjustment(leftX,ArmConstants.controllerDeadzone),2));
        //     }
        // }
    }

    public void moveElbowY(double leftY) //moves elbow joint up and down - assumed smallest encoder value is at bottom
    {
        double currentPosition = j2ElbowEncoder.getPosition();
        double maxValue = 1; //value of encoder at highest expected elbow position
        double minValue = 0; //value of encoder at lowest expected elbow position
        double speedLimiter = 0.5; //max motor voltage

        if( Math.abs(leftY) < 0.2 && minValue < currentPosition && currentPosition < maxValue )
        {
            j2_desiredPosition += (leftY * speedLimiter);
            j2_pid.setSetpoint(j2_desiredPosition);
        }

        // if( currentPosition > minValue && currentPosition < maxValue ) //prevents elbow from exceeding bounds
        // {
        //     if( leftY < 0 ) //lift elbow up - assumed run motor forwards
        //     {
        //         //runs motor at speedLimiter*x^2 where x is leftY //value is between 0-speedLimiter //squared to create curve
        //         j2Elbow.set(speedLimiter*Math.pow(getDeadZoneAdjustment(leftY,ArmConstants.controllerDeadzone),2));
        //         SmartDashboard.putNumber("Elbow Up/Down Force", speedLimiter*Math.pow(getDeadZoneAdjustment(leftY,ArmConstants.controllerDeadzone),2));
        //     }
        //     else if( leftY > 0 ) //lower elbow down - assumed run motor backwards
        //     {
        //         //runs motor at -speedLimiter*x^2 where x is leftY //value is between -speedLimiter-0 //squared to create curve
        //         j2Elbow.set(-speedLimiter*Math.pow(getDeadZoneAdjustment(leftY,ArmConstants.controllerDeadzone),2));
        //         SmartDashboard.putNumber("Elbow Up/Down Force", -speedLimiter*Math.pow(getDeadZoneAdjustment(leftY,ArmConstants.controllerDeadzone),2));
        //     }
        // }
    }

    public void moveWristY(double rightY) //moves wrist joint up and down - assumed smallest encoder value is at bottom
    {
        double currentPosition = j3WristYEncoder.getPosition();
        double maxValue = 1; //value of encoder at highest expected wrist position
        double minValue = 0; //value of encoder at lowest expected wrist position
        double speedLimiter = 0.5; //max motor voltage

        if( Math.abs(rightY) < 0.2 && minValue < currentPosition && currentPosition < maxValue )
        {
            j3_desiredPosition += (rightY * speedLimiter);
            j3_pid.setSetpoint(j3_desiredPosition);
        }

        // if( currentPosition > minValue && currentPosition < maxValue ) //prevents wrist from exceeding bounds
        // {
        //     if( rightY < 0 ) //lift wrist up - assumed run motor forwards
        //     {
        //         //runs motor at speedLimiter*x^2 where x is rightY //value is between 0-speedLimiter //squared to create curve
        //         j3WristY.set(speedLimiter*Math.pow(getDeadZoneAdjustment(rightY,ArmConstants.controllerDeadzone),2));
        //         SmartDashboard.putNumber("Wrist Up/Down Force", speedLimiter*Math.pow(getDeadZoneAdjustment(rightY,ArmConstants.controllerDeadzone),2));
        //     }
        //     else if( rightY > 0 ) //lower wrist down - assumed run motor backwards
        //     {
        //         //runs motor at -speedLimiter*x^2 where x is rightY //value is between -speedLimiter-0 //squared to create curve
        //         j3WristY.set(-speedLimiter*Math.pow(getDeadZoneAdjustment(rightY,ArmConstants.controllerDeadzone),2));
        //         SmartDashboard.putNumber("Wrist Up/Down Force", -speedLimiter*Math.pow(getDeadZoneAdjustment(rightY,ArmConstants.controllerDeadzone),2));
        //     }
        // }
    }

    public void openClaw(double leftTrigger) //opens claw - assumed run motor forwards
    {
        double currentPosition = j5ClawEncoder.getPosition();
        double maxValue = 1; //value of encoder at open-most position
        double minValue = 0; //value of encoder at close-most position
        double speedLimiter = 0.5; //max motor voltage

        if( Math.abs(leftTrigger) < 0.2 && minValue < currentPosition && currentPosition < maxValue )
        {
            j5_desiredPosition += (leftTrigger * speedLimiter);
            j5_pid.setSetpoint(j5_desiredPosition);
        }

        // if( currentPosition > minValue && currentPosition < maxValue ) //prevents claw from exceeding bounds
        // {
        //     //runs motor at speedLimiter*x^2 where x is leftTrigger //value is between 0-speedLimiter //squared to create curve
        //     j5Claw.set(speedLimiter*Math.pow(getDeadZoneAdjustment(leftTrigger,ArmConstants.controllerDeadzone),2));
        //     SmartDashboard.putNumber("Claw Open Force", speedLimiter*Math.pow(getDeadZoneAdjustment(leftTrigger,ArmConstants.controllerDeadzone),2));
        // }
    }

    public void closeClaw(double rightTrigger) //closes claw - assumed run motor backwards
    {
        double currentPosition = j5ClawEncoder.getPosition();
        double maxValue = 1; //value of encoder at open-most position
        double minValue = 0; //value of encoder at close-most position
        double speedLimiter = 0.5; //max motor voltage

        if( Math.abs(rightTrigger) < 0.2 && minValue < currentPosition && currentPosition < maxValue )
        {
            j5_desiredPosition += (rightTrigger * speedLimiter);
            j5_pid.setSetpoint(j5_desiredPosition);
        }

        // if( currentPosition > minValue && currentPosition < maxValue ) //prevents claw from exceeding bounds
        // {
        //     //runs motor at -speedLimiter*x^2 where x is leftX //value is between -speedLimiter-0 //squared to create curve
        //     j5Claw.set(-speedLimiter*Math.pow(getDeadZoneAdjustment(rightTrigger,ArmConstants.controllerDeadzone),2));
        //     SmartDashboard.putNumber("Claw Close Force", -speedLimiter*Math.pow(getDeadZoneAdjustment(rightTrigger,ArmConstants.controllerDeadzone),2));
        // }
    }

    public Command MoveArmToDefault()  //button_a pressed - default
    {
        j2_desiredPosition = 0;
        j3_desiredPosition = 0;
        return new PrintCommand("Default");
    }

    public Command MoveArmToBottom() //button_b pressed - bottom row
    {
        j2_desiredPosition = 0;
        j3_desiredPosition = 0;
        return new PrintCommand("Bottom");
    }

    public Command MoveArmToMiddle() //button_x pressed - middle row
    {
        j2_desiredPosition = 0;
        j3_desiredPosition = 0;
        return new PrintCommand("Middle");
    }

    public Command MoveArmToTop() //button_y pressed - top row
    {
        j2_desiredPosition = 0;
        j3_desiredPosition = 0;
        return new PrintCommand("Top");
    }


/**
* Adjust the value to provide a dead zone. The normal zone will be shorten zone that will allow
* a full 0 to 1/-1 range.
*
* @param orginalValue - The value to be adjusted.
* @param deadzone - The dead zone which should be a positive decimal representing a percentage.
* @return The newly adjusted value.
*/
public double getDeadZoneAdjustment(double orginalValue, double deadzone) {
    double maxRange = 1.0 - deadzone;
    double adjustedValue = orginalValue - (orginalValue * deadzone);
    if(isPositive(orginalValue))  // Value is positive.
    {
        // Verify that the adjusted value is still positive.
        if(isPositive(adjustedValue)) 
        {
            // We divide by the maxRange to give us the full range of 0 to 1.
            return adjustedValue / maxRange;
        }
    }
    else //Value is negative.
    {
        // Verify that the adjusted value is still negative.
        if(!isPositive(adjustedValue) && adjustedValue != 0) 
        {
            // We divide by the maxRange to give us the full range of 0 to 1.
            return adjustedValue / maxRange;
        }
    }
    
        /*
        *  The adjustment resulted in a sign change meaning the current value
        *  is in the dead zone. Therefore we are returning zero.
        */
        return 0;
    }
    
    /**
    * Returns true if the value is positive.
    *
    * @param value - The value to be checked.
    * @return true if the value is positive.
    */
    public boolean isPositive(double value) 
    {
        return value > 0;
    }
    // @Override
    // public double getMeasurement() {
    //     // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
    //     return  armEncoder.getRate();

    // // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=SOURCE
    // }

    // @Override
    // public void useOutput(double output, double setpoint) {
    //     output += setpoint*kF;
    //     // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
    // armMotor.set(output);

    // // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=OUTPUT
    // }

    

}
