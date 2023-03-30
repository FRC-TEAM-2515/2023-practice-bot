package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.util.OIReporters;
import frc.robot.util.RobotMath;
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
    private double j1Limiter = 0.5;
    private double j2Limiter = 0.4;
    private double j3Limiter = 0.2;
    private double j4Limiter = 0.1;


    public ArmCommand(Arm subsystem, XboxController controller) {
        armSubsystem = subsystem;
        armController = controller; 
        robotContainer = RobotContainer.getInstance();

        addRequirements(armSubsystem);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        getArmControllerDeadzone();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        armControlModeChoice = robotContainer.getOI().getArmControlModeChooser();
        armControllerScalingChoice = robotContainer.getOI().getControllerScalingChooser();

        double leftX = (robotContainer.getOI().getArmController().getLeftX());
        double leftY = (robotContainer.getOI().getArmController().getLeftY());
        double rightY = (robotContainer.getOI().getArmController().getRightY());

        double leftTrigger = (robotContainer.getOI().getArmController().getLeftTriggerAxis());
        double rightTrigger = (robotContainer.getOI().getArmController().getRightTriggerAxis());

        getArmControllerDeadzone();

        controllerScaling(leftX, leftY, rightY, armControllerScalingChoice);   
        armControlMode(leftX, leftY, rightY, leftTrigger, rightTrigger, armControlModeChoice);
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
            rightY = Math.copySign(rightY * rightY, rightY);

            OIReporters.ArmReporters.scalingMode = "Cubic";
            OIReporters.ArmReporters.cubicScaledX = ("leftX: " + RobotMath.truncate(leftX, 3) + "& leftY: " + RobotMath.truncate(leftY, 3));
            OIReporters.ArmReporters.cubicScaledY = ("rightY: " + RobotMath.truncate(rightY, 3));
            return;
        } 
            //non polynomic (fancy)
            leftX = leftX * 0.5 + Math.pow(3,(leftX * 0.5));
            leftY = leftY * 0.5 + Math.pow(3,(leftY * 0.5));
            rightY = leftX * 0.5 + Math.pow(3,(rightY * 0.5));
            
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
            double positionCommandJ1 = -j1ThrottleLeftX; //* Math.max(Arm.j1PaddingAddDeg, Arm.j1PaddingMinusDeg) / 2;
            double positionCommandJ2 = j2ThrottleLeftY; //* Math.max(Arm.j1PaddingAddDeg, Arm.j2PaddingMinusDeg);
            double positionCommandJ3 = -j3ThrottleRightY; // * j3Limiter;//Math.max(ArmConstants.kJ3AngleMax, -ArmConstants.kJ3AngleMin);
            double positionCommandOpenJ4 = leftTriggerOpen;//Math.max(ArmConstants.kJ4AngleMax, -ArmConstants.kJ4AngleMin);
            double positionCommandCloseJ4 = rightTriggerClose;//Math.max(ArmConstants.kJ4AngleMax, -ArmConstants.kJ4AngleMin);

            // double heading = Math.atan(positionCommandOpenJ4) * 720/ Math.PI + (Math.atan(positionCommandCloseJ4) * 720/ Math.PI);
            // SmartDashboard.putNumber("ArmTest/Heading", heading);

            double j1Heading = Math.atan(j1ThrottleLeftX) * 720/ Math.PI;
            SmartDashboard.putNumber("ArmTest/J1 Throttle Left Deg", j1Heading);

            double j2Heading = Math.tan(j1Heading) * Math.PI/ 720;
            SmartDashboard.putNumber("ArmTest/J1 Throttle Left Converted Raw", j2Heading);


            double positionCommandJ4 = 0;

            if (positionCommandOpenJ4 > ArmConstants.controllerDeadzone){
                positionCommandJ4 = positionCommandOpenJ4;
                 }
              if (positionCommandCloseJ4 > ArmConstants.controllerDeadzone){
                positionCommandJ4 = -positionCommandCloseJ4;
                 }  

            
        double limitedController = angleLimit(j1Heading, 1);
        SmartDashboard.putNumber("ArmTest/Limited Controller", limitedController);
        //     this.positionCommandJ2 = angleLimit(positionCommandJ2, Arm.j2PaddingAddDeg, Arm.j2PaddingMinusDeg);
        //     this.positionCommandJ3 = angleLimit(positionCommandJ3, Arm.j3PaddingAddDeg, Arm.j3PaddingMinusDeg);
        //    this.positionCommandOpenJ4 = angleLimit(positionCommandOpenJ4, Arm.j4PaddingMinusDeg, Arm.j1PaddingAddDeg);
        //    this.positionCommandCloseJ4 = angleLimit(positionCommandCloseJ4, Arm.j4PaddingMinusDeg, Arm.j4PaddingAddDeg);

			SmartDashboard.putNumber("ArmTest/J1 Joystick Command", (positionCommandJ1));
			SmartDashboard.putNumber("ArmTest/J2 Joystick Command", (positionCommandJ2));
			SmartDashboard.putNumber("ArmTest/J3 Joystick Command", (positionCommandJ3));
			SmartDashboard.putNumber("ArmTest/J4 Joystick Command", (positionCommandJ4));
			SmartDashboard.putNumber("ArmTest/J4 Joystick Open Command", (positionCommandOpenJ4));
			SmartDashboard.putNumber("ArmTest/J4 Joystick Close Command", (positionCommandCloseJ4));
        
            armSubsystem.manualControl((positionCommandJ1*j1Limiter),(positionCommandJ2*j2Limiter),(positionCommandJ3*j3Limiter),(positionCommandJ4*j4Limiter));
        }
    }

    public double deadzoneAdjustment(double controllerInput){
        
        if (Math.abs(controllerInput) < controllerDeadzone){
            controllerInput = 0;
        }
        return controllerInput;
    }

    public double angleLimit(double stickInputDeg, double joint){
       double controllerInputDeg = stickInputDeg;
    // if (joint == 1){
        double maxAngleDegLeft = ArmConstants.kJ1MaxLeftXControllerDeg;
        double maxAngleDegRight = ArmConstants.kJ1MaxRightXControllerDeg;
   // }
    if (controllerInputDeg > maxAngleDegLeft) {
        controllerInputDeg = maxAngleDegLeft;
    }
    if (controllerInputDeg < maxAngleDegRight) {
        controllerInputDeg = maxAngleDegRight;
    }
    SmartDashboard.putNumber("ArmTest/Limited Controller (deg)", maxAngleDegRight);
    return controllerInputDeg;
    //return RobotMath.controllerInputDegConvertRaw(controllerInputDeg);
    }

    public double getArmControllerDeadzone() {
        double rawArmControllerLeftX = Math.abs(RobotContainer.getInstance().getOI().getArmController().getLeftX());
        double rawArmControllerLeftY = Math.abs(RobotContainer.getInstance().getOI().getArmController().getLeftY());
        double rawArmControllerRightX = Math.abs(RobotContainer.getInstance().getOI().getArmController().getRightX());
        double rawArmControllerRightY = Math.abs(RobotContainer.getInstance().getOI().getArmController().getRightY());

        SmartDashboard.putNumber("ArmTest/raw left x", RobotMath.truncate(rawArmControllerLeftX, 3));
        SmartDashboard.putNumber("ArmTest/raw left y", RobotMath.truncate(rawArmControllerLeftY, 3));
        SmartDashboard.putNumber("ArmTest/raw right x", RobotMath.truncate(rawArmControllerRightX, 3));
        SmartDashboard.putNumber("ArmTest/raw right y", RobotMath.truncate(rawArmControllerRightY, 3));

        SmartDashboard.putNumber("ArmTest/og deadzone", controllerDeadzone);

       if (rawArmControllerLeftX > controllerDeadzone){
        controllerDeadzone = rawArmControllerLeftX;
       }
       if (rawArmControllerLeftY > controllerDeadzone){
        controllerDeadzone = rawArmControllerLeftY;
       }
       if (rawArmControllerRightX > controllerDeadzone){
        controllerDeadzone = rawArmControllerRightX;
       }
       if (rawArmControllerRightY > controllerDeadzone){
        controllerDeadzone = rawArmControllerRightY;
       }

        SmartDashboard.putNumber("ArmTest/new deadzone", controllerDeadzone);
        return controllerDeadzone;
       
    }
}

