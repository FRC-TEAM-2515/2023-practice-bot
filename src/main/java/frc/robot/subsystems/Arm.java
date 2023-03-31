package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.OI;
import frc.robot.RobotContainer;
import frc.robot.util.RobotMath;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.*;
import frc.robot.util.OIReporters;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ControlAffinePlantInversionFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MathUtil;

import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
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
private CANSparkMax j3Wrist;
private CANSparkMax j4Claw;

private SparkMaxAbsoluteEncoder j1TurretEncoder; 
private SparkMaxAbsoluteEncoder j2ElbowEncoder;
private SparkMaxAbsoluteEncoder j3WristEncoder;
private SparkMaxAbsoluteEncoder j4ClawEncoder;

// private RelativeEncoder j1TurretEncoder; 
// private SparkMaxAbsoluteEncoder j2ElbowEncoder;
// private RelativeEncoder j3WristYEncoder;
// private RelativeEncoder j4ClawEncoder;
// private RelativeEncoder j5ClawEncoder;
 
private double padding = 0;
private double kJ1RampRate = 4;
private double kJ2RampRate = 4;
private double kJ3RampRate = 4;
private double kJ4RampRate = 2;
public double controllerDeadzone = 0.2;
public enum armBrakeMode {Coast, Brake}; 

public boolean kCoast = false;

private SendableChooser<Arm.armBrakeMode> armBrakeModeChooser;

public double j1PaddingAddDeg, j2PaddingAddDeg, j3PaddingAddDeg, j4PaddingAddDeg;
public double j1PaddingMinusDeg, j2PaddingMinusDeg, j3PaddingMinusDeg, j4PaddingMinusDeg;


protected XboxController armController;

    public Arm() {

        armBrakeModeChooser = new SendableChooser<>();
         
        j1Turret = new CANSparkMax(11, MotorType.kBrushless); //10:1 MAG absolute encoder
        j2Elbow = new CANSparkMax(12, MotorType.kBrushless); //210:1 MAG absolute encoder
        j3Wrist = new CANSparkMax(13, MotorType.kBrushless); //5:1 MAG absolute encoder
        j4Claw = new CANSparkMax(15, MotorType.kBrushless); //built-in relative encoder

        j1Turret.restoreFactoryDefaults();  
        j1Turret.setInverted(false);
        j1Turret.setIdleMode(IdleMode.kBrake);
        //j1Turret.setIdleMode(IdleMode.kCoast);
        j1Turret.burnFlash();

        j2Elbow.restoreFactoryDefaults();  
        j2Elbow.setInverted(true);
        j2Elbow.setIdleMode(IdleMode.kBrake);
        //j2Elbow.setIdleMode(IdleMode.kCoast);
        j2Elbow.burnFlash();
        
        j3Wrist.restoreFactoryDefaults();  
        j3Wrist.setInverted(false);
        j3Wrist.setIdleMode(IdleMode.kBrake);
        //j3Wrist.setIdleMode(IdleMode.kCoast);
        j3Wrist.burnFlash();

        j4Claw.restoreFactoryDefaults();  
        j4Claw.setInverted(false);
        j4Claw.setIdleMode(IdleMode.kBrake);
        //j4Claw.setIdleMode(IdleMode.kCoast);
        j4Claw.burnFlash();

        j1TurretEncoder =  j1Turret.getAbsoluteEncoder(Type.kDutyCycle);
        j2ElbowEncoder =  j2Elbow.getAbsoluteEncoder(Type.kDutyCycle);
        j3WristEncoder =  j3Wrist.getAbsoluteEncoder(Type.kDutyCycle);
        j4ClawEncoder =  j4Claw.getAbsoluteEncoder(Type.kDutyCycle);
        
        // j1TurretEncoder = j1Turret.getEncoder();
        // j2ElbowEncoder =  j2Elbow.getAbsoluteEncoder(Type.kDutyCycle);
        // j3WristYEncoder =  j3WristY.getEncoder();
        // j4ClawEncoder =  j4Claw.getEncoder();
        // j5ClawEncoder =  j5Claw.getEncoder();        
    }

    public void manualControl(double j1JoystickInput,double j2JoystickInput ,double j3JoystickInput, double j4JoystickInput) {
      
         j1Turret.set(j1JoystickInput);
         j2Elbow.set(j2JoystickInput);
         j3Wrist.set(j3JoystickInput);
         j4Claw.set(j4JoystickInput);
      
    }
    

    public void updateSmartDashboard(){


        SmartDashboard.putNumber("j1TurretEncoder", j1TurretEncoder.getPosition());
        SmartDashboard.putNumber("j2ElbowEncoder", j2ElbowEncoder.getPosition());
        SmartDashboard.putNumber("j3WristYEncoder", j3WristEncoder.getPosition());
        SmartDashboard.putNumber("j4ClawEncoder", j4ClawEncoder.getPosition()); 

        SmartDashboard.putNumber("j1TurretEncoder D", RobotMath.armEncoderConvertDegrees(j1TurretEncoder.getPosition()));
        SmartDashboard.putNumber("j2ElbowEncoder D", RobotMath.armEncoderConvertDegrees(j2ElbowEncoder.getPosition()));
        SmartDashboard.putNumber("j3WristEncoder D", RobotMath.armEncoderConvertDegrees(j3WristEncoder.getPosition()));
        SmartDashboard.putNumber("j4ClawEncoder D", RobotMath.armEncoderConvertDegrees(j4ClawEncoder.getPosition()));

        armBrakeModeChooser.setDefaultOption("Brake", Arm.armBrakeMode.Brake);
        armBrakeModeChooser.addOption("Coast", Arm.armBrakeMode.Coast);


        if (Arm.armBrakeMode.Brake == armBrakeModeChooser.getSelected()){
            j1Turret.setIdleMode(IdleMode.kBrake);
            j2Elbow.setIdleMode(IdleMode.kBrake);
            j3Wrist.setIdleMode(IdleMode.kBrake);
            j4Claw.setIdleMode(IdleMode.kBrake);
            kCoast = false;
        }
        if (Arm.armBrakeMode.Coast == armBrakeModeChooser.getSelected()){
            j1Turret.setIdleMode(IdleMode.kCoast);
            j2Elbow.setIdleMode(IdleMode.kCoast);
            j3Wrist.setIdleMode(IdleMode.kCoast);
            j4Claw.setIdleMode(IdleMode.kCoast);
            kCoast = true;
        }
        SmartDashboard.putBoolean("Arm on Coast", kCoast);
        SmartDashboard.putData("Brake Mode", armBrakeModeChooser);
        
      

    }
    
    @Override
    public void periodic() {
        //super.periodic(); //needed if PID motion profiling Minussystem
        double j1RampRate = SmartDashboard.getNumber("J1 Ramp Rate", 4);
        double j2RampRate = SmartDashboard.getNumber("J2 Ramp Rate", 4);
        double j3RampRate = SmartDashboard.getNumber("J3 Ramp Rate", 4);
        double j4RampRate = SmartDashboard.getNumber("J4 Ramp Rate", 4);

        if((j1RampRate != kJ1RampRate)) { 
            kJ1RampRate = j1RampRate; }
        if((j2RampRate != kJ2RampRate)) {
            kJ2RampRate = j2RampRate; }
        if((j3RampRate != kJ3RampRate)) {
            kJ3RampRate = j3RampRate; }
        if((j4RampRate != kJ4RampRate)) {
            kJ4RampRate = j4RampRate; }
        
        SmartDashboard.putNumber("Applied J1 Ramp Rate", j1Turret.getOpenLoopRampRate());
        SmartDashboard.putNumber("Applied J2 Ramp Rate", j2Elbow.getOpenLoopRampRate());
        SmartDashboard.putNumber("Applied J3 Ramp Rate", j3Wrist.getOpenLoopRampRate());
        SmartDashboard.putNumber("Applied J4 Ramp Rate", j4Claw.getOpenLoopRampRate());

       updateSmartDashboard();
    }

    
    // public static double[] getPaddingArray1(){
    //     return kJointPadding1ArraySafe;
    // }

    
    // public static double[] getPaddingArray2(){
    //     return kJointPadding2ArraySafe;
    // }

    @Override
    public void simulationPeriodic() {
    }


    public enum getPaddingArray2 {
    }


    public enum paddingArray1 {
    }
}
