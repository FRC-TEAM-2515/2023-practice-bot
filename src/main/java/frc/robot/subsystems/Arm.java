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
 
private SlewRateLimiter armRateLimiter = new SlewRateLimiter(ArmConstants.armSlewRateLimiter);
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

public static double[] kJointPadding1ArraySafe = new double[] {};
public static double[] kJointPadding2ArraySafe = new double[] {};


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
      
        
        SmartDashboard.getNumber("J1 Ramp Rate", kJ1RampRate);
        SmartDashboard.getNumber("J2 Ramp Rate", kJ2RampRate);
        SmartDashboard.getNumber("J3 Ramp Rate", kJ3RampRate);
        SmartDashboard.getNumber("J4 Ramp Rate", kJ4RampRate);

        // j1Turret.setOpenLoopRampRate(kJ1RampRate);
        // j2Elbow.setOpenLoopRampRate(kJ2RampRate);
        // j3WristY.setOpenLoopRampRate(kJ3RampRate);
        // j4Claw.setOpenLoopRampRate(kJ4RampRate);
        // j5Claw.setOpenLoopRampRate(kJ5RampRate);

         j1Turret.set(j1JoystickInput);
         j2Elbow.set(j2JoystickInput);
         j3Wrist.set(j3JoystickInput);
         j4Claw.set(j4JoystickInput);
      
    }
    

    // public CommandBase unlimitedManualControl(double turnJ1Turret, double moveJ2Elbow, double rotatej3WristY, double rotatej4Claw, double openClaw, double closeClaw) {

    //     return run(() ->
    //     this.j1Turret.set(turnJ1Turret)).alongWith(run(() ->
    //     this.j2Elbow.set(moveJ2Elbow))).alongWith(run(() ->
	// 	this.j3WristY.set(rotatej3WristY))).alongWith(run(() ->
	// 	this.j4Claw.set(rotatej4Claw))).alongWith(run(() ->
    //     this.j5Claw.set(openClaw))).alongWith(run(() -> 
    //     this.j5Claw.set(-closeClaw)));
    // }

    public void updateSmartDashboard(){

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

        SmartDashboard.putBoolean("kCoast?", kCoast);
        SmartDashboard.putData("Brake Mode", armBrakeModeChooser);

        SmartDashboard.putNumber("j1TurretEncoder", j1TurretEncoder.getPosition());
        SmartDashboard.putNumber("j2ElbowEncoder", j2ElbowEncoder.getPosition());
        SmartDashboard.putNumber("j3WristYEncoder", j3WristEncoder.getPosition());
        SmartDashboard.putNumber("j4ClawEncoder", j4ClawEncoder.getPosition()); 

        SmartDashboard.putNumber("j1TurretEncoder D", RobotMath.armEncoderConvertDegrees(j1TurretEncoder.getPosition()));
        SmartDashboard.putNumber("j2ElbowEncoder D", RobotMath.armEncoderConvertDegrees(j2ElbowEncoder.getPosition()));
        SmartDashboard.putNumber("j3WristEncoder D", RobotMath.armEncoderConvertDegrees(j3WristEncoder.getPosition()));
        SmartDashboard.putNumber("j4ClawEncoder D", RobotMath.armEncoderConvertDegrees(j4ClawEncoder.getPosition()));

    this.padding = SmartDashboard.getNumber("RoM Safety Error", padding);
    SmartDashboard.putNumber("RoM Safety Error", padding);
    //if((romError != error)) { error = romError; }

    SmartDashboard.putNumber("J1 RoM w/ Safety Margin", RobotMath.armRangeDegrees(padding));
    SmartDashboard.putNumber("J2 RoM w/ Safety Margin", RobotMath.armRangeDegrees(padding));
    SmartDashboard.putNumber("J3 RoM w/ Safety Margin", RobotMath.armRangeDegrees(padding));
    SmartDashboard.putNumber("J4 RoM w/ Safety Margin", RobotMath.armRangeDegrees(padding) - 360);

    j1PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ1PaddingAddRot, padding);
    j1PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ1PaddingMinusRot, padding);
    SmartDashboard.putNumber("J1 Padding Add (deg)", j1PaddingAddDeg );
    SmartDashboard.putNumber("J1 Padding Minus (deg)", j1PaddingMinusDeg);

    j2PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ2PaddingAddRot, padding);
    j2PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ2PaddingMinusRot, padding);
    SmartDashboard.putNumber("J2 Padding Add (deg)", j2PaddingAddDeg);
    SmartDashboard.putNumber("J2 Padding Minus (deg)", RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ2PaddingMinusRot, padding));

    j3PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ3PaddingAddRot, padding);
    j3PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ3PaddingMinusRot, padding);
    SmartDashboard.putNumber("J3 Padding", RobotMath.armRangePaddingAddDeg(ArmConstants.kJ3PaddingAddRot, padding));
    SmartDashboard.putNumber("J3 Padding ", RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ3PaddingMinusRot, padding));

    j4PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.j4PaddingAddRot, 3);
    j4PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.j4PaddingMinusRot, 3);
    SmartDashboard.putNumber("j4ClawEncoder Padding1", ArmConstants.j4PaddingAddRot);
    SmartDashboard.putNumber("j4ClawEncoder Padding2", ArmConstants.j4PaddingAddRot);

    
    kJointPadding1ArraySafe = new double[] {j1PaddingAddDeg, j2PaddingAddDeg, j3PaddingAddDeg, j4PaddingMinusDeg};
    kJointPadding2ArraySafe = new double[] {j1PaddingMinusDeg, j1PaddingMinusDeg,j1PaddingMinusDeg, j4PaddingMinusDeg};

    SmartDashboard.putNumber("array J1 Max", kJointPadding1ArraySafe[1]);
    SmartDashboard.putNumber("array J2 Max", kJointPadding1ArraySafe[2]);
    SmartDashboard.putNumber("array J3 Max", kJointPadding1ArraySafe[3]);
    SmartDashboard.putNumber("array J4 Max", kJointPadding1ArraySafe[4]);
    
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

    
    public static double[] getPaddingArray1(){
        return kJointPadding1ArraySafe;
    }

    
    public static double[] getPaddingArray2(){
        return kJointPadding2ArraySafe;
    }

    @Override
    public void simulationPeriodic() {
    }
}
