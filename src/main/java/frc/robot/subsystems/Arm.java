package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.RobotMath;
import frc.robot.Constants.ArmConstants;
import frc.robot.OIReporters;

import frc.robot.commands.*;
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
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;  

/**
 *
 */
public class Arm extends SubsystemBase {

private CANSparkMax j1Turret; 
private CANSparkMax j2Elbow;
private CANSparkMax j3WristY;
private CANSparkMax j4WristX;
private CANSparkMax j5Claw;

// private SparkMaxAbsoluteEncoder j1TurretEncoder; 
// private SparkMaxAbsoluteEncoder j2ElbowEncoder;
// private RelativeEncoder j3WristYEncoder;
// private SparkMaxAbsoluteEncoder j4WristXEncoder;
// private SparkMaxAbsoluteEncoder j5ClawEncoder;


private RelativeEncoder j1TurretEncoder; 
private SparkMaxAbsoluteEncoder j2ElbowEncoder;
private RelativeEncoder j3WristYEncoder;
private RelativeEncoder j4WristXEncoder;
private RelativeEncoder j5ClawEncoder;

 /* Nonzero to block the config until success, zero to skip checking */
 final int kTimeoutMs = 30;
	
 /**
  * If the measured travel has a discontinuity, Note the extremities or
  * "book ends" of the travel.
  */
 final boolean kDiscontinuityPresent = true;
 final int kBookEnd_0 = 910;		/* 80 deg */
 final int kBookEnd_1 = 1137;	/* 100 deg */

 
 private SlewRateLimiter armRateLimiter = new SlewRateLimiter(ArmConstants.armSlewRateLimiter);
   
protected XboxController armController;

    public Arm() {
         
        j1Turret = new CANSparkMax(11, MotorType.kBrushless); //10:1 MAG absolute encoder
        j2Elbow = new CANSparkMax(12, MotorType.kBrushless); //210:1 MAG absolute encoder
        j3WristY = new CANSparkMax(13, MotorType.kBrushless); //5:1 MAG absolute encoder
        j4WristX = new CANSparkMax(14, MotorType.kBrushless); //built-in relative encoder
        j5Claw = new CANSparkMax(15, MotorType.kBrushless); //10:1

        j1Turret.restoreFactoryDefaults();  
        j1Turret.setInverted(false);
        j1Turret.setIdleMode(IdleMode.kBrake);
        j1Turret.burnFlash();

        j2Elbow.restoreFactoryDefaults();  
        j2Elbow.setInverted(true);
        j2Elbow.setIdleMode(IdleMode.kBrake);
        j2Elbow.burnFlash();
        
        j3WristY.restoreFactoryDefaults();  
        j3WristY.setInverted(false);
        j3WristY.setIdleMode(IdleMode.kBrake);
        j3WristY.burnFlash();

        j4WristX.restoreFactoryDefaults();  
        j4WristX.setInverted(false);
        j4WristX.setIdleMode(IdleMode.kBrake);
        j4WristX.burnFlash();

        j5Claw.restoreFactoryDefaults();  
        j5Claw.setInverted(false);
        j5Claw.setIdleMode(IdleMode.kBrake);
        j5Claw.burnFlash();

        // j1TurretEncoder =  j1Turret.getAbsoluteEncoder(Type.kDutyCycle);
        // j2ElbowEncoder =  j2Elbow.getAbsoluteEncoder(Type.kDutyCycle);
        // j3WristYEncoder =  j3WristY.getEncoder();
        // j4WristXEncoder =  j4WristX.getAbsoluteEncoder(Type.kDutyCycle);
        // j5ClawEncoder =  j5Claw.getAbsoluteEncoder(Type.kDutyCycle);
        
        j1TurretEncoder = j1Turret.getEncoder();
        j2ElbowEncoder =  j2Elbow.getAbsoluteEncoder(Type.kDutyCycle);
        j3WristYEncoder =  j3WristY.getEncoder();
        j4WristXEncoder =  j4WristX.getEncoder();
        j5ClawEncoder =  j5Claw.getEncoder();        
    }

    public CommandBase manualControl(double turnJ1Turret, double moveJ2Elbow, double rotatej3WristY, double rotatej4WristX, double openClaw, double closeClaw) {
        
        return run(() ->
        this.j1Turret.set(turnJ1Turret)).alongWith(run(() ->
        this.j2Elbow.set(moveJ2Elbow))).alongWith(run(() ->
		this.j3WristY.set(rotatej3WristY))).alongWith(run(() ->
		this.j4WristX.set(rotatej4WristX))).alongWith(run(() ->
        this.j5Claw.set(openClaw))).alongWith(run(() -> 
        this.j5Claw.set(-closeClaw)));
    }

    public void updateSmartDashboard(){

        SmartDashboard.putNumber("j1TurretEncoder", j1TurretEncoder.getPosition());
        SmartDashboard.putNumber("j2ElbowEncoder", j2ElbowEncoder.getPosition());
        SmartDashboard.putNumber("j3WristYEncoder", j3WristYEncoder.getPosition());
        SmartDashboard.putNumber("j4WristXEncoder", j4WristXEncoder.getPosition()); 
        SmartDashboard.putNumber("j5ClawEncoder", j5ClawEncoder.getPosition());

        SmartDashboard.putNumber("j1TurretEncoder D", RobotMath.j1EncoderConvertDegrees(j1TurretEncoder.getPosition()));
        SmartDashboard.putNumber("j2ElbowEncoder D", RobotMath.j2EncoderConvertDegrees(j2ElbowEncoder.getPosition()));
        SmartDashboard.putNumber("j3WristYEncoder D", RobotMath.j3EncoderConvertDegrees(j3WristYEncoder.getPosition()));
        SmartDashboard.putNumber("j4WristXEncoder D", RobotMath.j4EncoderConvertDegrees(j4WristXEncoder.getPosition())); 
        SmartDashboard.putNumber("j5ClawEncoder D", RobotMath.j5EncoderConvertDegrees(j5ClawEncoder.getPosition()));
    }

    @Override
    public void periodic() {
        //super.periodic(); //needed for PID motion profiling
       updateSmartDashboard();
    
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

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

    public double getAngle() {
        return 0;
      }

    public Object rotateArm(double output) {
        return null;
    }

}
