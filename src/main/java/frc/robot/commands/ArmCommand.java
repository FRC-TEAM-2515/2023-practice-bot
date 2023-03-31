package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Arm.paddingArray1;
import frc.robot.util.OIReporters;
import frc.robot.util.RobotMath;
import frc.robot.util.Vector2;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.*;
import frc.robot.Constants;

import frc.robot.util.OIReporters.ArmControlType;

import frc.robot.util.OIReporters.ControllerScaling;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.*;
import java.util.function.DoubleSupplier;
import frc.robot.OI;

public class ArmCommand extends CommandBase{
    
    private Arm armSubsystem; 
    private RobotContainer robotContainer;
    private XboxController armController;
    private OIReporters.ArmControlType armControlModeChoice;
    private OIReporters.ControllerScaling armControllerScalingChoice;
    private double leftX;
    private double leftY;
    private double rightY;
    private double leftTrigger;
    private double rightTrigger;
    private double positionCommandJ1;
    private double positionCommandJ2;
    private double positionCommandJ3;    
    private double positionCommandOpenJ4;
    private double positionCommandCloseJ4;
    private double positionCommandJ4;
    private double controllerDeadzone = 0.2;
    private double j1Limiter = 0.35;
    private double j2Limiter = 0.4;
    private double j3Limiter = 0.2;
    private double j4Limiter = 0.2;
    private double j1PaddingAddDeg;
    private double j1PaddingMinusDeg;
    private double j2PaddingMinusDeg;
    private double j2PaddingAddDeg;
    private double j3PaddingMinusDeg;
    private double j3PaddingAddDeg;
    private double padding;
    private double paddingAddArray1[];
    private double paddingMinusArray2[];


    public ArmCommand(Arm subsystem, XboxController controller) {
        armSubsystem = subsystem;
        armController = controller; 
        robotContainer = RobotContainer.getInstance();

        addRequirements(armSubsystem);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        armControlModeChoice = robotContainer.getOI().getArmControlModeChooser();
        armControllerScalingChoice = robotContainer.getOI().getControllerScalingChooser();

        double leftX = robotContainer.getOI().getArmController().getLeftX();
        double leftY = robotContainer.getOI().getArmController().getLeftY();
        double rightY = robotContainer.getOI().getArmController().getRightY();
        double rightX = robotContainer.getOI().getArmController().getRightX();

        double leftTrigger = robotContainer.getOI().getArmController().getLeftTriggerAxis();
        double rightTrigger = robotContainer.getOI().getArmController().getRightTriggerAxis();

        Vector2 leftStickAxialDZ = deadzoneAdjustment(leftX, leftY);
        Vector2 rightStickAxialDZ = deadzoneAdjustment(rightX, rightY);

        double j1LeftX = (float) leftStickAxialDZ.x;
        OIReporters.ArmReporters.ControllerScaled[1] = j1LeftX;
        OIReporters.ArmReporters.ControllerRaw[1] = leftX;

        double j2LeftY = (float) leftStickAxialDZ.y;
        OIReporters.ArmReporters.ControllerScaled[2] = j2LeftY;
        OIReporters.ArmReporters.ControllerRaw[2] = leftY;

        double j3RightY = (float) rightStickAxialDZ.y;
        OIReporters.ArmReporters.ControllerScaled[3] = j3RightY;
        OIReporters.ArmReporters.ControllerRaw[3] = rightY;
        
        
        controllerScaling(leftX, leftY, rightY, armControllerScalingChoice);
        armControlMode(j1LeftX, j2LeftY, j3RightY, leftTrigger, rightTrigger, armControlModeChoice);
    }

    public void controllerScaling(double leftX, double leftY, double rightY, ControllerScaling choice){

        if (choice == ControllerScaling.LINEAR){ //linear scaling
            this.leftX = leftX; 
            this.leftY = leftY;
            this.rightY = rightY;

            OIReporters.ArmReporters.scalingMode = "Linear";
            OIReporters.ArmReporters.linearScaledX = ("leftX: " + RobotMath.truncate(leftX, 3) + " & leftY: " + RobotMath.truncate(leftY, 3));
            OIReporters.ArmReporters.linearScaledY =  ("rightY: " + RobotMath.truncate(rightY, 3));
            return;
        }
        if (choice == ControllerScaling.SQUARED) { //squared scaling
            leftX = Math.copySign(leftX * leftX, leftX);
            leftY = Math.copySign(leftY * leftY, leftY);
            rightY = Math.copySign(rightY * rightY, rightY);
                
            OIReporters.ArmReporters.scalingMode = "Squared";
            OIReporters.ArmReporters.squaredScaledX = ("leftX: " + RobotMath.truncate(leftX, 3) + "& leftY: " + RobotMath.truncate(leftY, 3));
            OIReporters.ArmReporters.squaredScaledY = ("rightY: " + RobotMath.truncate(rightY, 3));
            return;
        }
        if (choice == ControllerScaling.CUBIC) { //cubic scaling
            leftX = leftX * leftX * leftX;
            leftY = leftY * leftY * leftY;
            rightY = rightY * rightY * rightY;

            OIReporters.ArmReporters.scalingMode = "Cubic";
            OIReporters.ArmReporters.cubicScaledX = ("leftX: " + RobotMath.truncate(leftX, 3) + "& leftY: " + RobotMath.truncate(leftY, 3));
            OIReporters.ArmReporters.cubicScaledY = ("rightY: " + RobotMath.truncate(rightY, 3));
            return;
        } 
            //non polynomic (fancy)
            leftX = leftX * 0.5 + Math.pow(3,(leftX * 0.5));
            leftY = leftY * 0.5 + Math.pow(3,(leftY * 0.5));
            rightY = rightY * 0.5 + Math.pow(3,(rightY * 0.5));
            
            OIReporters.ArmReporters.scalingMode = "Fancy";
            OIReporters.ArmReporters.fancyScaledX = ("leftX: " + RobotMath.truncate(leftX, 3) + "& leftY: " + RobotMath.truncate(leftY, 3));
            OIReporters.ArmReporters.fancyScaledY = ("rightY: " + RobotMath.truncate(rightY, 3));
    }
    
    public void armControlMode(double j1ThrottleLeftX, double j2ThrottleLeftY, double j3ThrottleRightY, double leftTriggerOpen, double rightTriggerClose, OIReporters.ArmControlType choice){

        // if (choice == ArmControlType.VELOCITY){
        //     double velocityCommandJ1 = -throttleRightX * ArmConstants.kJ1DegPerSecMax;
        //     double velocityCommandJ2 = -throttleRightY * ArmConstants.kJ2DegPerSecMax;
        //     double velocityCommandJ3 = -throttleLeftY  * ArmConstants.kJ3DegPerSecMax;
        //     double velocityCommandJ4 = -throttleLeftX  * ArmConstants.kJ4DegPerSecMax;
        // }

        if (choice == ArmControlType.POSITION){

            double positionCommandJ1 = -j1ThrottleLeftX * j1Limiter ;
            double positionCommandJ2 = j2ThrottleLeftY * j2Limiter;
            double positionCommandJ3 = -j3ThrottleRightY * j3Limiter; 
            double positionCommandOpenJ4 = leftTriggerOpen;
            double positionCommandCloseJ4 = rightTriggerClose;

            double positionCommandJ4 = 0;

            if (positionCommandOpenJ4 > ArmConstants.controllerDeadzone){
                positionCommandJ4 = positionCommandOpenJ4;
                 }
              if (positionCommandCloseJ4 > ArmConstants.controllerDeadzone){
                positionCommandJ4 = -positionCommandCloseJ4;
                 }  
            
            double j1LimitedController = limitArmController(j1ThrottleLeftX,1);

    j1PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ1PaddingAddRot, padding);
    j1PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ1PaddingMinusRot, padding);
    SmartDashboard.putNumber("J1 Padding Add (deg)", j1PaddingAddDeg );
    SmartDashboard.putNumber("J1 Padding Minus (deg)", j1PaddingMinusDeg);

    j2PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ2PaddingAddRot, padding);
    double j2LimitedController = limitArmController(j2ThrottleLeftY, 2);
    SmartDashboard.putNumber("j2 limited controller", j2LimitedController);
    double j3LimitedController = limitArmController(j3ThrottleRightY, 3);
    SmartDashboard.putNumber("j3 limited controller", j3LimitedController);aPaddingAddDeg(ArmConstants.kJ2PaddingAddRot, padding);
    j2PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ2PaddingMinusRot, padding);
    SmartDashboard.putNumber("J2 Padding Add (deg)", j2PaddingAddDeg);
    SmartDashboard.putNumber("J2 Padding Minus (deg)", RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ2PaddingMinusRot, padding));

    j3PaddingAddDeg = RobotMath.armRangePaddingAddDeg(ArmConstants.kJ3PaddingAddRot, padding);
    j3PaddingMinusDeg = RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ3PaddingMinusRot, padding);
    SmartDashboard.putNumber("J3 Padding", RobotMath.armRangePaddingAddDeg(ArmConstants.kJ3PaddingAddRot, padding));
    SmartDashboard.putNumber("J3 Padding ", RobotMath.armRangePaddingMinusDeg(ArmConstants.kJ3PaddingMinusRot, padding));}

    
       
    }

    private void aPaddingAddDeg(double kj2paddingaddrot, double padding2) {
    }

    public double limitArmController (double stickInput, int joint){
        double maxLeftDeg = 0;
        double maxRightDeg = 0;
        double inputDegrees = RobotMath.controllerInputRawConvertDeg(stickInput);

        if (joint == 1){
            maxLeftDeg = ArmConstants.kJ1ControllerMaxLeft8;
            maxRightDeg = ArmConstants.kJ1ControllerMaxRight8;
        }   
        if (joint == 2){
            maxLeftDeg = ArmConstants.kJ1ControllerMaxLeft8;
            maxRightDeg = ArmConstants.kJ1ControllerMaxRight8;
        }
        if (joint == 3){
            maxLeftDeg = ArmConstants.kJ1ControllerMaxLeft8;
            maxRightDeg = ArmConstants.kJ1ControllerMaxRight8;
        }

        if (inputDegrees > maxLeftDeg) {
            inputDegrees = maxLeftDeg;
        }

        if (inputDegrees > maxRightDeg){
            inputDegrees = maxLeftDeg;
        }
            
        return inputDegrees;
        
    }

    public double angleLimit(double positionCommand, int joint){
        double maxLeftDeg = 0;
        double maxRightDeg = 0;
        double inputDegrees = positionCommand;

        if (joint == 1){
            maxLeftDeg = ArmConstants.kJ1PaddingAddRot;
            maxRightDeg = ArmConstants.kJ1PaddingMinusRot;
        }   
        if (joint == 2){
            maxLeftDeg = ArmConstants.kJ2PaddingAddRot;
            maxRightDeg = ArmConstants.kJ2PaddingMinusRot;
        }
        if (joint == 3){
            maxLeftDeg = ArmConstants.kJ3PaddingAddRot;
            maxRightDeg = ArmConstants.kJ3PaddingMinusRot;
        }

        if (inputDegrees > maxLeftDeg) {
            inputDegrees = maxLeftDeg;
        }

        if (inputDegrees > maxRightDeg){
            inputDegrees = maxLeftDeg;
        }
            
        return inputDegrees;
        
    }
    


    public Vector2 deadzoneAdjustment(double j1ThrottleLeftX, double j2ThrottleLeftY){
    double x = j1ThrottleLeftX;
    float j1x = (float) x;

    double y = j2ThrottleLeftY;
    float j2y = (float) y;

    float deadzone = 0.75f;

    Vector2 stickInput = new Vector2((j1x), (j2y));
    if(Math.abs(stickInput.x) < deadzone){
        stickInput.x = 0.0f;
    }
    if(Math.abs(stickInput.y) < deadzone){
        stickInput.y = 0.0f;
    }
    return stickInput;
    }

}


