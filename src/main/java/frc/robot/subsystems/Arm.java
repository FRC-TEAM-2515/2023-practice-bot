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
public static double j1PaddingAddDeg, j2PaddingAddDeg, j3PaddingAddDeg, j4PaddingAddDeg;
public static double j1PaddingMinusDeg, j2PaddingMinusDeg, j3PaddingMinusDeg, j4PaddingMinusDeg;

public static double[] kPadding1ArraySafe = new double[] {j1PaddingAddDeg, j2PaddingAddDeg, j3PaddingAddDeg, j4PaddingAddDeg};
public static double[] kPadding2ArraySafe = new double[] {j1PaddingMinusDeg, j2PaddingMinusDeg, j3PaddingMinusDeg, j4PaddingMinusDeg};


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

    SmartDashboard.putNumber("J1 RoM w/ Safety Margin", RobotMath.armRangeDeg(padding, ArmConstants.kJ1PaddingAddRot, ArmConstants.kJ1PaddingMinusRot));
    SmartDashboard.putNumber("J2 RoM w/ Safety Margin", RobotMath.armRangeDeg(padding, ArmConstants.kJ2PaddingAddRot, ArmConstants.kJ2PaddingMinusRot));
    SmartDashboard.putNumber("J3 RoM w/ Safety Margin", RobotMath.armRangeDeg(padding, ArmConstants.kJ3PaddingAddRot, ArmConstants.kJ3PaddingMinusRot));
    SmartDashboard.putNumber("J4 RoM w/ Safety Margin", RobotMath.armRangeDeg(3, ArmConstants.kJ4PaddingAddRot, ArmConstants.kJ4PaddingMinusRot) - 360);

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
    SmartDashboard.putNumber("J3 Padding Add (deg)", RobotMath.armRangePaddingAddDeg(ArmConstants.kJ3PaddingAddRot, padding));
    SmartDashboard.putNumber("J3 Padding Minus (deg)", RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ3PaddingMinusRot, padding));

    j4PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ4PaddingAddRot, 3);
    j4PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ4PaddingMinusRot, 3);
    SmartDashboard.putNumber("J4 Padding Add (deg)", j4PaddingAddDeg);
    SmartDashboard.putNumber("j4ClawEncoder Padding2", j4PaddingMinusDeg);

    kPadding1ArraySafe = new double[] {j1PaddingAddDeg, j2PaddingAddDeg, j3PaddingAddDeg, j4PaddingMinusDeg};
    kPadding2ArraySafe = new double[] {j1PaddingMinusDeg, j1PaddingMinusDeg,j1PaddingMinusDeg, j4PaddingMinusDeg};

    
    // SmartDashboard.putNumber("J1 RoM w/ Safety Margin", RobotMath.truncate(Math.abs(RobotMath.armRangeErrorMarginAdd(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ1PaddingAdd), error) + RobotMath.armRangeErrorMarginMinus(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ1PaddingMinus), error) -360), 3));
    // SmartDashboard.putNumber("J2 RoM w/ Safety Margin", RobotMath.truncate(Math.abs(RobotMath.armRangeErrorMarginAdd(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ2PaddingAdd), error) + RobotMath.armRangeErrorMarginMinus(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ2PaddingMinus), error) - 360), 3));
    // SmartDashboard.putNumber("J3 RoM w/ Safety Margin", RobotMath.truncate(Math.abs(RobotMath.armRangeErrorMarginAdd(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ3PaddingAdd), error) + RobotMath.armRangeErrorMarginMinus(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ3PaddingMinus), error) - 360), 3));
    // SmartDashboard.putNumber("J4 RoM w/ Safety Margin", RobotMath.truncate(Math.abs(RobotMath.armRangeErrorMarginAdd(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ4PaddingAdd), error) + RobotMath.armRangeErrorMarginMinus(RobotMath.armEncoderConvertDegrees(ArmConstants.kJ4PaddingMinus), error) - 360), 3));

    j1PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ1PaddingAddRot, padding);
    j4PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ1PaddingMinusRot, padding);
    SmartDashboard.putNumber("j1TurretEncoder Padding1", j1PaddingAddDeg );
    SmartDashboard.putNumber("j1TurretEncoder Padding2", j1PaddingMinusDeg);

    j2PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ2PaddingAddRot, padding);
    j2PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ2PaddingMinusRot, padding);
    SmartDashboard.putNumber("j2ElbowEncoder Padding1", j2PaddingAddDeg);
    SmartDashboard.putNumber("j2ElbowEncoder Padding2", RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ2PaddingMinusRot, padding));

    j3PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ3PaddingAddRot, padding);
    j3PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ3PaddingMinusRot, padding);
    SmartDashboard.putNumber("j3WristEncoder Padding1", RobotMath.armRangePaddingAddDeg(ArmConstants.kJ3PaddingAddRot, padding));
    SmartDashboard.putNumber("j3WristEncoder Padding2", RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ3PaddingMinusRot, padding));

    j4PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ4PaddingAddRot, 3);
    j4PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ4PaddingMinusRot, 3);
    SmartDashboard.putNumber("j4ClawEncoder Padding1", ArmConstants.kJ4PaddingAddRot);
    SmartDashboard.putNumber("j4ClawEncoder Padding2", ArmConstants.kJ4PaddingAddRot);

    
    // kPaddingPadding1ArraySafe = new double[] {j1PaddingAddDeg, j2PaddingAddDeg, j3PaddingAddDeg, j4PaddingMinusDeg};
    // kPaddingPadding2ArraySafe = new double[] {j1PaddingMinusDeg, j4PaddingMinusDeg,j4PaddingMinusDeg, j1PaddingAddDeg};

    }
    
    @Override
    public void periodic() {
        //super.periodic(); //needed if PID motion profiling subsystem
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
        return kPadding1ArraySafe;
    }

    
    public static double[] getPaddingArray2(){
        return kPadding2ArraySafe;
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run when in simulation

//     }
//     public void moveShoulder(double shoulderForce){
//         double shoulderLimiter = .5;
//         j2Elbow.set(getDeadZoneAdjustment(shoulderForce*shoulderLimiter,ArmConstants.controllerDeadzone));
//         SmartDashboard.putNumber("Shoulder Force", getDeadZoneAdjustment(shoulderForce,ArmConstants.controllerDeadzone));
//     }
//     public void moveElbow(double elbowForce){
//         double elbowLimiter = .20;
//         j3WristY.set(getDeadZoneAdjustment(elbowForce*elbowLimiter,ArmConstants.controllerDeadzone));
//         SmartDashboard.putNumber("Elbow Force", getDeadZoneAdjustment(elbowForce,ArmConstants.controllerDeadzone));
//     }    
//     public void moveClaw(double open, double close){
//         // armMotor.set(force);
//         double clawForce = 0;
//         double clawLimiter = .10;
//         if (open > ArmConstants.controllerDeadzone){
//             clawForce = open;
//           }
//           if (close > ArmConstants.controllerDeadzone){
//             clawForce = -close;
//           }  
//         j5Claw.set(getDeadZoneAdjustment(clawForce*clawLimiter,ArmConstants.controllerDeadzone));
//         SmartDashboard.putNumber("Claw Force", getDeadZoneAdjustment(clawForce,ArmConstants.controllerDeadzone));
//     }  
//     public void rotateWaist(double waistForce){
//         double waistLimiter = .50;
//         j1Turret.set(getDeadZoneAdjustment(waistForce*waistLimiter,waistForce));
//         SmartDashboard.putNumber("Waist Force", getDeadZoneAdjustment(waistForce,ArmConstants.controllerDeadzone));
//     }
//     public void rotateWrist(double wristForce){
//         double wristLimiter = .05;
//         j4Claw.set(getDeadZoneAdjustment(wristForce*wristLimiter,ArmConstants.controllerDeadzone));
//         SmartDashboard.putNumber("Wrist Force", getDeadZoneAdjustment(wristForce,ArmConstants.controllerDeadzone));
//     }
// /**
// * Adjust the value to provide a dead zone. The normal zone will be shorten zone that will allow
// * a full 0 to 1/-1 range.
// *
// * @param orginalValue - The value to be adjusted.
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
    
//     /*
//     *  The adjustment resulted in a sign change meaning the current value
//     *  is in the dead zone. Therefore we are returning zero.
//     */
//     return 0;
//     }
    
//     /**
//     * Returns true if the value is positive.
//     *
//     * @param value - The value to be checked.
//     * @return true if the value is positive.
//     */
//     public boolean isPositive(double value) {
//     return value > 0;
//     }
   
   

    

}
}
