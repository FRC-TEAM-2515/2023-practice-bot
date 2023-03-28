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
        double rightX = RobotContainer.getInstance().getOI().getArmController().getRightX();
        double rightY =RobotContainer.getInstance().getOI().getArmController().getRightY();

        double rightTrigger = RobotContainer.getInstance().getOI().getArmController().getRightTriggerAxis();
        double leftTrigger = RobotContainer.getInstance().getOI().getArmController().getLeftTriggerAxis();
        boolean leftBumper = RobotContainer.getInstance().getOI().getArmController().getLeftBumper();
        boolean rightBumper = RobotContainer.getInstance().getOI().getArmController().getRightBumper();
        boolean button_x = RobotContainer.getInstance().getOI().getArmController().getXButtonPressed();
        boolean button_y = RobotContainer.getInstance().getOI().getArmController().getYButtonPressed();
        boolean button_a = RobotContainer.getInstance().getOI().getArmController().getAButtonPressed();
        boolean button_b = RobotContainer.getInstance().getOI().getArmController().getBButtonPressed();


        moveElbowY(leftY);
        moveWristY(rightY);
        moveElbowX(leftX);
        openClaw(leftTrigger);
        closeClaw(rightTrigger);


        // moveArm(rightY);
        // rotateElbow(rightX);
        // moveClaw(rightTrigger);
        position1(button_a);
        position2(button_b);
        position3(button_x);
        position4(button_y);
        //rotateWrist(leftBumper, rightBumper);
        // rotateWaist(leftX);
        // moveShoulder(leftY);
        // moveElbow(rightY);
        // rotateWrist(rightX);
        // moveClaw(leftTrigger, rightTrigger);
    }
    

    // public CommandBase unlimitedManualControl(double turnJ1Turret, double moveJ2Elbow, double rotatej3WristY, double rotatej4WristX, double openClaw, double closeClaw) {

    //     return run(() ->
    //     this.j1Turret.set(turnJ1Turret)).alongWith(run(() ->
    //     this.j2Elbow.set(moveJ2Elbow))).alongWith(run(() ->
	// 	this.j3WristY.set(rotatej3WristY))).alongWith(run(() ->
	// 	this.j4WristX.set(rotatej4WristX))).alongWith(run(() ->
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
    
    }

    @Override
    public void simulationPeriodic()
    {
        // This method will be called once per scheduler run when in simulation

    }

    public void moveElbowX(double leftX)
    {
        double currentPosition = j1TurretEncoder.getPosition();
        double maxRightValue = 1; //value of encoder at rightmost turret position
        double maxLeftValue = 0; //value of encoder at leftmost turret position
        double speedLimiter = 0.5; //limits speed of motor

        if( currentPosition > maxRightValue && currentPosition < maxLeftValue ) //prevents turret from exceeding bounds
        {
            if( leftX < 0 ) //rotates turret to left - assumed run motor forwards
            {
                j1Turret.set(Math.pow(getDeadZoneAdjustment(leftX*speedLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Turret Rotation Force", getDeadZoneAdjustment(leftX,ArmConstants.controllerDeadzone));
            }
            else if( leftX > 0 ) //rotates turret to right - assumed run motor backwards
            {
                j1Turret.set(Math.pow(getDeadZoneAdjustment(leftX*speedLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Turret Rotation Force", getDeadZoneAdjustment(leftX,ArmConstants.controllerDeadzone));
            }
        }
    }

    public void moveElbowY(double leftY)
    {
        double currentPosition = j2ElbowEncoder.getPosition();
        double maxValue = 1; //value of encoder at highest expected elbow position
        double minValue = 0; //value of encoder at lowest expected elbow position
        double speedLimiter = 0.5; //limits speed of motor

        if( currentPosition > minValue && currentPosition < maxValue ) //prevents elbow from exceeding bounds
        {
            if( leftY < 0 ) //lift elbow up - assumed run motor forwards
            {
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(leftY*speedLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(leftY,ArmConstants.controllerDeadzone));
            }
            else if( leftY > 0 ) //lower elbow down - assumed run motor backwards
            {
                j2Elbow.set(-Math.pow(getDeadZoneAdjustment(leftY*speedLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(leftY,ArmConstants.controllerDeadzone));
            }
        }
    }

    public void moveWristY(double rightY)
    {
        double currentPosition = j3WristYEncoder.getPosition();
        double maxValue = 1; //value of encoder at highest expected wrist position
        double minValue = 0; //value of encoder at lowest expected wrist position
        double speedLimiter = 0.5; //limits speed of motor

        if( currentPosition > minValue && currentPosition < maxValue ) //prevents wrist from exceeding bounds
        {
            if( rightY < 0 ) //lift wrist up - assumed run motor forwards
            {
                j3WristY.set(Math.pow(getDeadZoneAdjustment(rightY*speedLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Up/Down Force", getDeadZoneAdjustment(rightY,ArmConstants.controllerDeadzone));
            }
            else if( rightY > 0 ) //lower wrist down - assumed run motor backwards
            {
                j3WristY.set(-Math.pow(getDeadZoneAdjustment(rightY*speedLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Up/Down Force", getDeadZoneAdjustment(rightY,ArmConstants.controllerDeadzone));
            }
        }
    }

    public void openClaw(double leftTrigger) //opens claw - assumed run motor forwards
    {
        double currentPosition = j5ClawEncoder.getPosition();
        double maxValue = 1; //value of encoder at open-most position
        double minValue = 0; //value of encoder at close-most position
        double speedLimiter = 0.5; //limits speed of motor

        if( currentPosition > minValue && currentPosition < maxValue ) //prevents claw from exceeding bounds
        {
            j5Claw.set(Math.pow(getDeadZoneAdjustment(leftTrigger*speedLimiter,ArmConstants.controllerDeadzone),2));
            SmartDashboard.putNumber("Claw Open Force", getDeadZoneAdjustment(leftTrigger,ArmConstants.controllerDeadzone));
        }
    }

    public void closeClaw(double rightTrigger) //closes claw - assumed run motor backwards
    {
        double currentPosition = j5ClawEncoder.getPosition();
        double maxValue = 1; //value of encoder at open-most position
        double minValue = 0; //value of encoder at close-most position
        double speedLimiter = 0.5; //limits speed of motor

        if( currentPosition > minValue && currentPosition < maxValue ) //prevents claw from exceeding bounds
        {
            j5Claw.set(-Math.pow(getDeadZoneAdjustment(rightTrigger*speedLimiter,ArmConstants.controllerDeadzone),2));
            SmartDashboard.putNumber("Claw Close Force", getDeadZoneAdjustment(-rightTrigger,ArmConstants.controllerDeadzone));
        }
    }

    // public void moveArm(double armForce)    //moves the elbow and wrist up and down
    // {
    //     double wristPosition = j3WristYEncoder.getPosition();
    //     double elbowPosition = j2ElbowEncoder.getPosition();
    //     double elbowLimiter = 0.5;
    //     double wristLimiter = 0.20;
    //     double maxWristValue = 1;   //highest wrist encoder value
    //     double minWristValue = 0.5; //lowest wrist encoder value
    //     double maxElbowValue = 1;   //highest elbow encoder value
    //     double minElbowValue = 0;   //lowest elbow encoder value


    //     if( armForce < 0 ) //pulling backward on right stick - move arm up
    //     {
    //         if( wristPosition < maxWristValue ) //lift wrist up
    //         {
    //             j3WristY.set(Math.pow(getDeadZoneAdjustment(armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
    //             SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
    //         }
    //         else if( elbowPosition < maxElbowValue )   //lift elbow up
    //         {
    //             j2Elbow.set(Math.pow(getDeadZoneAdjustment(armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
    //             SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
    //         }
    //     }
    //     else if( armForce > 0 ) //pushing forward on right stick - move arm down
    //     {
    //         if( wristPosition > minWristValue ) //lower wrist down
    //         {
    //             j3WristY.set(Math.pow(getDeadZoneAdjustment(armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
    //             SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
    //         }
    //         else if( elbowPosition > minElbowValue )    //lower elbow down
    //         {
    //             j2Elbow.set(Math.pow(getDeadZoneAdjustment(armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
    //             SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
    //         }
    //     }
    // }

    // public void rotateElbow(double elbowForce)  //rotates elbow (turret)
    // {
    //     double waistLimiter = .50;
    //     j1Turret.set(Math.pow(getDeadZoneAdjustment(elbowForce*waistLimiter,elbowForce),2));
    //     SmartDashboard.putNumber("Elbow Rotate Force", getDeadZoneAdjustment(elbowForce,ArmConstants.controllerDeadzone));
    // }

    // public void moveClaw(double moveForce)  //opens/close claw
    // {
    //     double currentPosition = j5ClawEncoder.getPosition();
    //     double maxCloseValue = 0;       //value of encoder when claw is fully closed
    //     double minCloseValue = 0.5;     //value of encoder when claw is closed enough to hold piece
    //     double maxOpenValue = 1;        //value of encoder when claw is fully open
    //     double minOpenValue = 0.5;      //value of encoder when claw is open enough to let go of piece
    //     //double midValue = (closeValue + openValue) / 2; //threshold between open and closed
    //     double clawLimiter = .10;
    //     boolean currentState;       //true if open, false if closed
        
    //     if( minCloseValue > currentPosition && currentPosition > maxCloseValue )
    //         currentState = false;
    //     else if( minOpenValue < currentPosition && currentPosition < maxOpenValue )
    //         currentState = true;
    //     else
    //         currentState = true;

    //     if( !currentState ) //if claw is closed
    //     {
    //         //open claw at speed of moveForce;
    //         j5Claw.set(Math.pow(getDeadZoneAdjustment(moveForce*clawLimiter,ArmConstants.controllerDeadzone),2));
    //         SmartDashboard.putNumber("Claw Force", getDeadZoneAdjustment(moveForce,ArmConstants.controllerDeadzone));
    //     }
    //     else //if claw is open
    //     {
    //         //close claw at speed of -moveForce;
    //         j5Claw.set(Math.pow(getDeadZoneAdjustment(-moveForce*clawLimiter,ArmConstants.controllerDeadzone),2));
    //         SmartDashboard.putNumber("Claw Force", getDeadZoneAdjustment(-moveForce,ArmConstants.controllerDeadzone));
    //     }
    // }

    public void position1(boolean pressed)  //button_a pressed - default
    {
        double wristPosition = j3WristYEncoder.getPosition();
        double elbowPosition = j2ElbowEncoder.getPosition();
        double desiredWristPosition = 1;
        double desiredElbowPosition = 1;
        double elbowLimiter = 0.5;
        double wristLimiter = 0.20;
        double armForce = 0.5;
        while( wristPosition != desiredWristPosition )
        {
            wristPosition = j3WristYEncoder.getPosition();
            if( wristPosition < desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( wristPosition > desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(-armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }

        while( elbowPosition != desiredElbowPosition )
        {
            elbowPosition = j2ElbowEncoder.getPosition();
            if( elbowPosition < desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( elbowPosition > desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(-armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }
    }

    public void position2(boolean pressed) //button_b pressed - bottom row
    {
        double wristPosition = j3WristYEncoder.getPosition();
        double elbowPosition = j2ElbowEncoder.getPosition();
        double desiredWristPosition = 1;
        double desiredElbowPosition = 1;
        double elbowLimiter = 0.5;
        double wristLimiter = 0.20;
        double armForce = 0.5;
        while( wristPosition != desiredWristPosition )
        {
            wristPosition = j3WristYEncoder.getPosition();
            if( wristPosition < desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( wristPosition > desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(-armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }
        
        while( elbowPosition != desiredElbowPosition )
        {
            elbowPosition = j2ElbowEncoder.getPosition();
            if( elbowPosition < desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( elbowPosition > desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(-armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }
    }

    public void position3(boolean pressed) //button_x pressed - middle row
    {
        double wristPosition = j3WristYEncoder.getPosition();
        double elbowPosition = j2ElbowEncoder.getPosition();
        double desiredWristPosition = 1;
        double desiredElbowPosition = 1;
        double elbowLimiter = 0.5;
        double wristLimiter = 0.20;
        double armForce = 0.5;
        while( wristPosition != desiredWristPosition )
        {
            wristPosition = j3WristYEncoder.getPosition();
            if( wristPosition < desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( wristPosition > desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(-armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }
        
        while( elbowPosition != desiredElbowPosition )
        {
            elbowPosition = j2ElbowEncoder.getPosition();
            if( elbowPosition < desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( elbowPosition > desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(-armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }
    }

    public void position4(boolean pressed) //button_y pressed - top row
    {
        double wristPosition = j3WristYEncoder.getPosition();
        double elbowPosition = j2ElbowEncoder.getPosition();
        double desiredWristPosition = 1;
        double desiredElbowPosition = 1;
        double elbowLimiter = 0.5;
        double wristLimiter = 0.20;
        double armForce = 0.5;
        while( wristPosition != desiredWristPosition )
        {
            wristPosition = j3WristYEncoder.getPosition();
            if( wristPosition < desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( wristPosition > desiredWristPosition )
            {
                //move wrist to position
                j3WristY.set(Math.pow(getDeadZoneAdjustment(-armForce*wristLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Wrist Y Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }
        
        while( elbowPosition != desiredElbowPosition )
        {
            elbowPosition = j2ElbowEncoder.getPosition();
            if( elbowPosition < desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
            else if( elbowPosition > desiredElbowPosition )
            {
                //move elbow to position
                j2Elbow.set(Math.pow(getDeadZoneAdjustment(-armForce*elbowLimiter,ArmConstants.controllerDeadzone),2));
                SmartDashboard.putNumber("Elbow Up/Down Force", getDeadZoneAdjustment(armForce,ArmConstants.controllerDeadzone));
            }
        }
    }

    // public void rotateWrist(boolean leftWristForce, boolean rightWristForce) //rotates wrist
    // {
    //     double currentPosition = j4WristXEncoder.getPosition();
    //     double wristLimiter = 0.05;
    //     double wristForce = 0.5;    //speed at which the wrist will rotate
    //     double leftStop = 1;        //encoder value for furthest the wrist can rotate left
    //     double rightStop = -1;       //encoder value for furthest the wrist can rotate right
    //     if(leftWristForce && !rightWristForce && currentPosition < leftStop )
    //     {
    //         //rotate wrist left
    //         j4WristX.set(Math.pow(getDeadZoneAdjustment(wristForce*wristLimiter,ArmConstants.controllerDeadzone),2));
    //         SmartDashboard.putNumber("Wrist Force", getDeadZoneAdjustment(wristForce,ArmConstants.controllerDeadzone));
    //     }
    //     else if(!leftWristForce && rightWristForce && currentPosition > rightStop )
    //     {
    //         //rotate wrist right
    //         j4WristX.set(Math.pow(getDeadZoneAdjustment(-wristForce*wristLimiter,ArmConstants.controllerDeadzone),2));
    //         SmartDashboard.putNumber("Wrist Force", getDeadZoneAdjustment(wristForce,ArmConstants.controllerDeadzone));
    //     }
    // }

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
