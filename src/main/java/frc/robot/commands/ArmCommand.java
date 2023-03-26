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
        
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

        m_armControlModeChoice = m_robotContainer.getOI().getArmControlModeChooser();
        m_armControllerScalingChoice = m_robotContainer.getOI().getControllerScalingChooser();

        double leftX = RobotContainer.getInstance().getOI().getArmController().getLeftX();
        double leftY = RobotContainer.getInstance().getOI().getArmController().getLeftY();
        double rightX = RobotContainer.getInstance().getOI().getArmController().getRightX();
        double rightY = RobotContainer.getInstance().getOI().getArmController().getRightY();

        double leftTrigger = RobotContainer.getInstance().getOI().getArmController().getLeftTriggerAxis();
        double rightTrigger = RobotContainer.getInstance().getOI().getArmController().getRightTriggerAxis();

        controllerScaling(leftX, leftY, rightX, rightY, m_armControlModeChoice);   
        
        
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

    public void armControlMode(double throttleLeftX, double throttleLeftY, double throttleRightX, double throttleRightY, Enum choice){

        if (choice == ArmControlType.VELOCITY){
            double velocityCommandJ1 = -throttleRightX * ArmConstants.kJ1DegPerSecMax;
            double velocityCommandJ2 = -throttleRightY * ArmConstants.kJ2DegPerSecMax;
            double velocityCommandJ3 = -throttleLeftY  * ArmConstants.kJ3DegPerSecMax;
            double velocityCommandJ4 = -throttleLeftX  * ArmConstants.kJ4DegPerSecMax;
        }
    }

    

}
