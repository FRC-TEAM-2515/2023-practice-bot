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

        // double leftX = deadzoneAdjustment(robotContainer.getOI().getArmController().getLeftX());
        // double leftY = deadzoneAdjustment(robotContainer.getOI().getArmController().getLeftY());
        // double rightY = deadzoneAdjustment(robotContainer.getOI().getArmController().getRightY());


        double leftX = robotContainer.getOI().getArmController().getLeftX();
        double leftY = robotContainer.getOI().getArmController().getLeftY();
        double rightY = robotContainer.getOI().getArmController().getRightY();


        double leftTrigger = (robotContainer.getOI().getArmController().getLeftTriggerAxis());
        double rightTrigger = (robotContainer.getOI().getArmController().getRightTriggerAxis());

       // getArmControllerDeadzone();

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
            double positionCommandJ1 = j1ThrottleLeftX ;//Math.max(ArmConstants.kJ1AngleMax, -ArmConstants.kJ1AngleMin) / 2;
            double positionCommandJ2 = j2ThrottleLeftY ;//Math.max(ArmConstants.kJ2AngleMax, -ArmConstants.kJ2AngleMin);
            double positionCommandJ3 = -j3ThrottleRightY; // * j3Limiter;//Math.max(ArmConstants.kJ3AngleMax, -ArmConstants.kJ3AngleMin);
            double positionCommandOpenJ4 = leftTriggerOpen;//Math.max(ArmConstants.kJ4AngleMax, -ArmConstants.kJ4AngleMin);
            double positionCommandCloseJ4 = rightTriggerClose;//Math.max(ArmConstants.kJ4AngleMax, -ArmConstants.kJ4AngleMin);

            double positionCommandJ4 = 0;

            if (positionCommandOpenJ4 > ArmConstants.controllerDeadzone){
                positionCommandJ4 = positionCommandOpenJ4;
                 }
              if (positionCommandCloseJ4 > ArmConstants.controllerDeadzone){
                positionCommandJ4 = -positionCommandCloseJ4;
                 }  

            
            // this.positionCommandJ1 = angleLimit(positionCommandJ1, 1);
            // this.positionCommandJ2 = angleLimit(positionCommandJ2, 2);
            // this.positionCommandJ3 = angleLimit(positionCommandJ3, 3);
           // this.positionCommandOpenJ4 = angleLimit(positionCommandOpenJ4, 4);
           // this.positionCommandCloseJ4 = angleLimit(positionCommandCloseJ4, 4);

			SmartDashboard.putNumber("J1 Joystick Command", RobotMath.truncate(positionCommandJ1, 3));
			SmartDashboard.putNumber("J2 Joystick Command", RobotMath.truncate(positionCommandJ2, 3));
			SmartDashboard.putNumber("J3 Joystick Command", RobotMath.truncate(positionCommandJ3, 3));
			SmartDashboard.putNumber("J4 Joystick Command", RobotMath.truncate(positionCommandJ4, 3));
			SmartDashboard.putNumber("J4 Joystick Open Command", RobotMath.truncate(positionCommandOpenJ4, 3));
			SmartDashboard.putNumber("J4 Joystick Close Command", RobotMath.truncate(positionCommandCloseJ4, 3));
        
            armSubsystem.manualControl(positionCommandJ1*j1Limiter,positionCommandJ2*j2Limiter,positionCommandJ3*j3Limiter,(positionCommandJ4*j4Limiter));
        }
    }

    public double deadzoneAdjustment(double controllerInput){
        
        if (Math.abs(controllerInput) < controllerDeadzone){
            controllerInput = 0;
        }
        return controllerInput;
    }

    // public double angleLimit(double positionCommand, int joint){
    // //    double jointBookendArray1[] = Arm.getPaddingArray1();
    // //    double jointBookendArray2[] = Arm.getPaddingArray2();

    //   double maxAngleDeg = jointBookendArray1 [joint];
    //   double minAngleDeg = jointBookendArray2 [joint];

    // if (positionCommand > maxAngleDeg) {
    //     positionCommand = maxAngleDeg;
    // }
    // if (positionCommand < minAngleDeg) {
    //     positionCommand = minAngleDeg;
    // }
    // return positionCommand;
    // }

    public double getArmControllerDeadzone() {
        double rawArmControllerLeftX = Math.abs(RobotContainer.getInstance().getOI().getArmController().getLeftX());
        double rawArmControllerLeftY = Math.abs(RobotContainer.getInstance().getOI().getArmController().getLeftY());
        double rawArmControllerRightX = Math.abs(RobotContainer.getInstance().getOI().getArmController().getRightX());
        double rawArmControllerRightY = Math.abs(RobotContainer.getInstance().getOI().getArmController().getRightY());

        SmartDashboard.putNumber("raw left x", RobotMath.truncate(rawArmControllerLeftX, 3));
        SmartDashboard.putNumber("raw left y", RobotMath.truncate(rawArmControllerLeftY, 3));
        SmartDashboard.putNumber("raw right x", RobotMath.truncate(rawArmControllerRightX, 3));
        SmartDashboard.putNumber("raw right y", RobotMath.truncate(rawArmControllerRightY, 3));

        SmartDashboard.putNumber("og deadzone", controllerDeadzone);

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

        SmartDashboard.putNumber("new deadzone", controllerDeadzone);
        return controllerDeadzone;
       
    }
}

