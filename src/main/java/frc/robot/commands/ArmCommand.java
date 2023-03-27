package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Arm;
import frc.robot.util.OIReporters;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.*;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.RobotContainer.*;
import java.util.function.DoubleSupplier;
import frc.robot.OI;

public class ArmCommand extends CommandBase{
    
    private Arm m_armSubsystem; 
    private RobotContainer m_robotContainer;
    private XboxController m_armController;
    private Enum m_armControlModeChoice;
    private Enum m_armControllerScalingChoice;
    private double leftX;
    private double leftY;
    private double rightX; 
    private double rightY;
    private double leftTrigger;
    private double rightTrigger;
    private double positionCommandJ1;
    private double positionCommandJ2;
    private double positionCommandJ3;
    private double positionCommandJ4;
    private double positionCommandOpenJ5;
    private double positionCommandCloseJ5;
    private double controllerDeadzone = 0.2;

    public ArmCommand(Arm subsystem, XboxController controller) {
        m_armSubsystem = subsystem;
        m_armController = controller; 
        m_robotContainer = RobotContainer.getInstance();

        // Ensures that two commands that need the same subsystem dont mess each other up. 
        addRequirements (m_armSubsystem);  
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        getArmControllerDeadzone();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        m_armControlModeChoice = m_robotContainer.getOI().getArmControlModeChooser();
        m_armControllerScalingChoice = m_robotContainer.getOI().getControllerScalingChooser();

        double leftX = deadzoneAdjustment(m_robotContainer.getOI().getArmController().getLeftX());
        double leftY = deadzoneAdjustment(m_robotContainer.getOI().getArmController().getLeftY());
        double rightX = deadzoneAdjustment(m_robotContainer.getOI().getArmController().getRightX());
        double rightY = deadzoneAdjustment(m_robotContainer.getOI().getArmController().getRightY());

        double leftTrigger = deadzoneAdjustment(m_robotContainer.getOI().getArmController().getLeftTriggerAxis());
        double rightTrigger = deadzoneAdjustment(m_robotContainer.getOI().getArmController().getRightTriggerAxis());

        controllerScaling(leftX, leftY, rightX, rightY, m_armControllerScalingChoice);   
        armControlMode(leftX, leftY, rightX, rightY, leftTrigger, rightTrigger, m_armControlModeChoice);
    }

    public void controllerScaling(double leftX, double leftY, double rightX, double rightY, Enum choice){

        if (choice == ControllerScaling.LINEAR){ //linear scaling
            this.leftX = leftX; 
            this.leftY = leftY;
            this.rightX = rightX;
            this.rightY = rightY;

            OIReporters.ArmReporters.scalingMode = "Linear";
            OIReporters.ArmReporters.linearScaledX = ("leftX: " + leftX + "& leftY: " + leftY);
            OIReporters.ArmReporters.linearScaledY = ("rightX: " + rightX + "& rightY: " + rightY);
            return;
        }
        if (choice == ControllerScaling.SQUARED) { //squared scaling
            leftX = Math.copySign(leftX * leftX, leftX);
            leftY = Math.copySign(leftY * leftY, leftY);
            rightX = Math.copySign(rightX * rightX, rightX);
            rightY = Math.copySign(rightY * rightY, rightY);
                
            OIReporters.ArmReporters.scalingMode = "Squared";
            OIReporters.ArmReporters.squaredScaledX = ("leftX: " + leftX + "& leftY: " + leftY);
            OIReporters.ArmReporters.squaredScaledY = ("rightX: " + rightX + "& rightY: " + rightY);
            return;
        }
        if (choice == ControllerScaling.CUBIC) { //cubic scaling
            leftX = leftX * leftX * leftX;
            leftY = leftY * leftY * leftY;
            rightX = Math.copySign(rightX * rightX, rightX);
            rightY = Math.copySign(rightY * rightY, rightY);

            OIReporters.ArmReporters.scalingMode = "Cubic";
            OIReporters.ArmReporters.cubicScaledX = ("leftX: " + leftX + "& leftY: " + leftY);
            OIReporters.ArmReporters.cubicScaledY = ("rightX: " + rightX + "& rightY: " + rightY);
            return;
        } 
            //non polynomic (fancy)
            leftX = leftX * 0.5 + Math.pow(3,(leftX * 0.5));
            leftY = leftY * 0.5 + Math.pow(3,(leftY * 0.5));
            rightX = rightX * 0.5 + Math.pow(3,(rightX * 0.5));
            rightY = leftX * 0.5 + Math.pow(3,(rightY * 0.5));
            
            OIReporters.ArmReporters.scalingMode = "Fancy";
            OIReporters.ArmReporters.fancyScaledX = ("leftX: " + leftX + "& leftY: " + leftY);
            OIReporters.ArmReporters.fancyScaledY = ("leftX: " + leftX + "& rightY: " + rightY);
    }
    
    public void armControlMode(double throttleLeftX, double throttleLeftY, double throttleRightX, double throttleRightY, double leftTriggerOpen, double rightTriggerClose, Enum choice){

        // if (choice == ArmControlType.VELOCITY){
        //     double velocityCommandJ1 = -throttleRightX * ArmConstants.kJ1DegPerSecMax;
        //     double velocityCommandJ2 = -throttleRightY * ArmConstants.kJ2DegPerSecMax;
        //     double velocityCommandJ3 = -throttleLeftY  * ArmConstants.kJ3DegPerSecMax;
        //     double velocityCommandJ4 = -throttleLeftX  * ArmConstants.kJ4DegPerSecMax;
        // }
        if (choice == ArmControlType.POSITION){
            double positionCommandJ1 = -throttleRightX * Math.max(ArmConstants.kJ1AngleMax, -ArmConstants.kJ1AngleMin) / 2;
            double positionCommandJ2 = -throttleRightY * Math.max(ArmConstants.kJ2AngleMax, -ArmConstants.kJ2AngleMin);
            double positionCommandJ3 = -throttleLeftY  * Math.max(ArmConstants.kJ3AngleMax, -ArmConstants.kJ3AngleMin);
            double positionCommandJ4 = -throttleLeftX  * Math.max(ArmConstants.kJ4AngleMax, -ArmConstants.kJ4AngleMin);
            double positionCommandOpenJ5 = -leftTriggerOpen * Math.max(ArmConstants.kJ5AngleMax, -ArmConstants.kJ5AngleMin);
            double positionCommandCloseJ5 = rightTriggerClose * Math.max(ArmConstants.kJ5AngleMax, -ArmConstants.kJ5AngleMin);

            this.positionCommandJ1 = angleLimit(positionCommandJ1, 1);
            this.positionCommandJ2 = angleLimit(positionCommandJ2, 2);
            this.positionCommandJ3 = angleLimit(positionCommandJ3, 3);
            this.positionCommandJ4 = angleLimit(positionCommandJ4, 4);
            this.positionCommandOpenJ5 = angleLimit(positionCommandOpenJ5, 5);
            this.positionCommandCloseJ5 = angleLimit(positionCommandCloseJ5, 5);

			SmartDashboard.putNumber("J1 Joystick Command", positionCommandJ1);
			SmartDashboard.putNumber("J2 Joystick Command", positionCommandJ2);
			SmartDashboard.putNumber("J3 Joystick Command", positionCommandJ3);
			SmartDashboard.putNumber("J4 Joystick Command", positionCommandJ4);
			SmartDashboard.putNumber("J5 Joystick Open Command", positionCommandOpenJ5);
			SmartDashboard.putNumber("J5 Joystick Close Command", positionCommandCloseJ5);
        }
    }

    public double deadzoneAdjustment(double controllerInput){
        
        if (Math.abs(controllerInput) < controllerDeadzone){
            controllerInput = 0;
        }
        return controllerInput;
    }

    public double angleLimit(double positionCommand, int joint){
      double maxAngleDeg = ArmConstants.kJointAngleMaxArray[joint];
      double minAngleDeg = ArmConstants.kJointAngleMinArray[joint];

    if (positionCommand > maxAngleDeg) {
        positionCommand = maxAngleDeg;
    }
    if (positionCommand < minAngleDeg) {
        positionCommand = minAngleDeg;
    }
    return positionCommand;
    }

    public double getArmControllerDeadzone() {
        double rawArmControllerLeftX = Math.abs(RobotContainer.getInstance().getOI().getArmController().getLeftX());
        double rawArmControllerLeftY = Math.abs(RobotContainer.getInstance().getOI().getArmController().getLeftY());
        double rawArmControllerRightX = Math.abs(RobotContainer.getInstance().getOI().getArmController().getRightX());
        double rawArmControllerRightY = Math.abs(RobotContainer.getInstance().getOI().getArmController().getRightY());

        SmartDashboard.putNumber("raw left x", rawArmControllerLeftX);
        SmartDashboard.putNumber("raw left y", rawArmControllerLeftY);
        SmartDashboard.putNumber("raw right x", rawArmControllerRightX);
        SmartDashboard.putNumber("raw right y", rawArmControllerRightY);

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

