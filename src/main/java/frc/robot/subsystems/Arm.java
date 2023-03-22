package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.RobotMath;
import frc.robot.Constants.ArmConstants;
import frc.robot.OIReporters;

import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxAnalogSensor;
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
private CANSparkMax j3WristX;
private CANSparkMax j4WristY;
private CANSparkMax j5Claw;

private SparkMaxAbsoluteEncoder j1TurretEncoder; 
private SparkMaxAbsoluteEncoder j2ElbowEncoder;
private SparkMaxAbsoluteEncoder j3WristXEncoder;
private SparkMaxAnalogSensor j4WristYEncoder;
private SparkMaxAbsoluteEncoder j5ClawEncoder;

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
        j3WristX = new CANSparkMax(13, MotorType.kBrushless); //5:1 MAG absolute encoder
        j4WristY = new CANSparkMax(14, MotorType.kBrushless); //built-in relative encoder
        j5Claw = new CANSparkMax(15, MotorType.kBrushless); //10:1

        j1Turret.restoreFactoryDefaults();  
        j1Turret.setInverted(false);
        j1Turret.setIdleMode(IdleMode.kBrake);
        j1Turret.burnFlash();

        j2Elbow.restoreFactoryDefaults();  
        j2Elbow.setInverted(true);
        j2Elbow.setIdleMode(IdleMode.kBrake);
        j2Elbow.burnFlash();
        
        j3WristX.restoreFactoryDefaults();  
        j3WristX.setInverted(false);
        j3WristX.setIdleMode(IdleMode.kBrake);
        j3WristX.burnFlash();

        j4WristY.restoreFactoryDefaults();  
        j4WristY.setInverted(false);
        j4WristY.setIdleMode(IdleMode.kBrake);
        j4WristY.burnFlash();

        j5Claw.restoreFactoryDefaults();  
        j5Claw.setInverted(false);
        j5Claw.setIdleMode(IdleMode.kBrake);
        j5Claw.burnFlash();

        j1TurretEncoder =  j1Turret.getAbsoluteEncoder(Type.kDutyCycle);
        j2ElbowEncoder =  j2Elbow.getAbsoluteEncoder(Type.kDutyCycle);
        j3WristXEncoder =  j3WristX.getAbsoluteEncoder(Type.kDutyCycle);
        j4WristYEncoder =  j4WristY.getAnalog(SparkMaxAnalogSensor.Mode.kAbsolute);
        j5ClawEncoder =  j5Claw.getAbsoluteEncoder(Type.kDutyCycle);
    }

    public void manualControl() {
        double turnJ1Turret = RobotContainer.getInstance().getOI().getArmController().getLeftX();
        double moveJ2Elbow = RobotContainer.getInstance().getOI().getArmController().getLeftY();
        double rotateJ3WristX = RobotContainer.getInstance().getOI().getArmController().getRightX();
        double rotateJ4WristY = RobotContainer.getInstance().getOI().getArmController().getRightY();

        j1Turret.set(turnJ1Turret);
		j2Elbow.set(moveJ2Elbow);
		j3WristX.set(rotateJ3WristX);
		j4WristY.set(rotateJ4WristY);
    }
    

    @Override
    public void periodic() {
        //super.periodic();

        manualControl();
        
        SmartDashboard.putNumber("j1TurretEncoder", j1TurretEncoder.getPosition());
        SmartDashboard.putNumber("j2ElbowEncoder", j2ElbowEncoder.getPosition());
        SmartDashboard.putNumber("j3WristX", j3WristXEncoder.getPosition());
        SmartDashboard.putNumber("j4WristY Encoder", j4WristYEncoder.getPosition()); // 1 volt per rev
        SmartDashboard.putNumber("j5Claw", j5ClawEncoder.getPosition());

        
        
        SmartDashboard.putNumber("j1TurretEncoder D", RobotMath.j1EncoderConvertDegrees(j1TurretEncoder.getPosition()));
        SmartDashboard.putNumber("j2ElbowEncoder D", RobotMath.j2EncoderConvertDegrees(j2ElbowEncoder.getPosition()));
        SmartDashboard.putNumber("j3WristX D", RobotMath.j3EncoderConvertDegrees(j3WristXEncoder.getPosition()));
        SmartDashboard.putNumber("j4WristY Encoder D", RobotMath.j4EncoderConvertDegrees(j4WristYEncoder.getPosition())); // 1 volt per rev
        SmartDashboard.putNumber("j5Claw D", RobotMath.j5EncoderConvertDegrees(j5ClawEncoder.getPosition()));
    
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

    // public void moveShoulder(double shoulderForce){
    //     double shoulderLimiter = .5;
    //     shoulderMotor.set(getDeadZoneAdjustment(shoulderForce*shoulderLimiter,deadzone));
    //     SmartDashboard.putNumber("Shoulder Force", getDeadZoneAdjustment(shoulderForce,deadzone));
    // }
    // public void moveElbow(double elbowForce){
    //     double elbowLimiter = .20;
    //     elbowMotor.set(getDeadZoneAdjustment(elbowForce*elbowLimiter,deadzone));
    //     SmartDashboard.putNumber("Elbow Force", getDeadZoneAdjustment(elbowForce,deadzone));
    // }    
    // public void moveClaw(double open, double close){
    //     // armMotor.set(force);
    //     double clawForce = 0;
    //     double clawLimiter = .10;
    //     if (open > deadzone){
    //         clawForce = open;
    //       }
    //       if (close > deadzone){
    //         clawForce = -close;
    //       }  
    //     clawMotor.set(getDeadZoneAdjustment(clawForce*clawLimiter,deadzone));
    //     SmartDashboard.putNumber("Claw Force", getDeadZoneAdjustment(clawForce,deadzone));
    // }  
    // public void rotateWaist(double waistForce){
    //     double waistLimiter = .50;
    //     waistMotor.set(getDeadZoneAdjustment(waistForce*waistLimiter,waistForce));
    //     SmartDashboard.putNumber("Waist Force", getDeadZoneAdjustment(waistForce,deadzone));
    // }
    // public void rotateWrist(double wristForce){
    //     double wristLimiter = .05;
    //     wristMotor.set(getDeadZoneAdjustment(wristForce*wristLimiter,deadzone));
    //     SmartDashboard.putNumber("Wrist Force", getDeadZoneAdjustment(wristForce,deadzone));
    // }

//     * @param orginalValue - The value to be adjusted.
// * @param deadzone - The dead zone which should be a positive decimal representing a percentage.
// * @return The newly adjusted value.
// */
// public double getDeadZoneAdjustment(double orginalValue, double deadzone) {
//     double maxRange = 1.0 - deadzone;
//     double adjustedValue = orginalValue - (orginalValue * deadzone);
//     if(isPositive(orginalValue)) { // Value is positive.
//     // Verify that the adjusted value is still positive.
//     if(isPositive(adjustedValue)) {
//     // We divide by the maxRange to give us the full range of 0 to 1.
//     return adjustedValue / maxRange;
//     }
//     } else { //Value is negative.
//     // Verify that the adjusted value is still negative.
//     if(!isPositive(adjustedValue) && adjustedValue != 0) {
//     // We divide by the maxRange to give us the full range of 0 to 1.
//     return adjustedValue / maxRange;
//     }
//     }
    
}
