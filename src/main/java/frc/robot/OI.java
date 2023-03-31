package frc.robot;

import frc.robot.util.OIReporters.ArmControlType;
import frc.robot.util.OIReporters.AutoCommand;
import frc.robot.util.OIReporters.ControllerScaling;
import frc.robot.util.OIReporters.DriveControllerMode;
import frc.robot.util.OIReporters.DriveType;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoDriveBackward;
//import frc.robot.commands.auto.SimpleAutonomous;
import frc.robot.subsystems.*;
import frc.robot.util.OIReporters;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.RobotMath;
import frc.robot.Constants.ArmConstants;

public class OI {
    
    private static OI instanceOI;
    private OIReporters oiReporters;
    private XboxController m_driveController;
    private XboxController m_armController;

    private CommandXboxController m_commandDriveController;
    private Trigger resetButton;
    private JoystickButton brakeButton;

    private SendableChooser<AutoCommand> autoChooser;

    private SendableChooser<DriveControllerMode> driverControlsChooser;
    private SendableChooser<ControllerScaling> controllerScalingChooser;
    private SendableChooser<DriveType> driveTypeChooser;

    private SendableChooser<ArmControlType> armControlModeChooser;
    private SendableChooser<ControllerScaling> armControllerScalingChooser;
   
    //private SimpleAutonomous simpleAutonomous;
    //private AutoDriveBackward that you were going to at least attempt something automated.
    

  // public static OI getInstance() {
  //     if (instanceOI == null) {
  //       instanceOI = new OI();
  //     }
  //     return instanceOI;
  //   }

    public OI() {

    // Controllers
    m_driveController = new XboxController(0);
    m_commandDriveController = new CommandXboxController(0);
    m_armController = new XboxController(1);

    // Command choosers
    autoChooser = new SendableChooser<>();

    // Object choosers
    driverControlsChooser = new SendableChooser<>();
    controllerScalingChooser = new SendableChooser<>();
    driveTypeChooser = new SendableChooser<>();

    armControlModeChooser = new SendableChooser<>();
    armControllerScalingChooser = new SendableChooser<>();

    configureButtonBindings();
    configureSmartDashboard();
    getDrivePreferences();
    getArmPreferences();

}
public void initOI() {
    configureButtonBindings();
    configureSmartDashboard();
    getDrivePreferences();
}

private void configureButtonBindings() {

    shouldEnableBrakes();
    shouldInvertMotors();

    m_commandDriveController.a().onTrue(new SafeReset());
    
  
}

public boolean shouldInvertMotors() {
    return m_driveController.getAButtonPressed();
  }

public boolean shouldEnableBrakes() {
    return m_driveController.getBButton();
}



public void configureSmartDashboard() {

    // Choosers
    autoChooser.setDefaultOption("Competition/Simple Autonomous", AutoCommand.SIMPLE);

    driverControlsChooser.setDefaultOption("Competition/Left Stick", DriveControllerMode.LEFT_STICK);
    driverControlsChooser.addOption("Competition/Trigger Acceleration", DriveControllerMode.TRIGGER_ACCEL);
    
    controllerScalingChooser.addOption("Competition/Limited Polynomic", ControllerScaling.LIMITED_POLYNOMIC);
    controllerScalingChooser.addOption("Competition/Linear", ControllerScaling.LINEAR);
    controllerScalingChooser.addOption("Competition/Squared", ControllerScaling.SQUARED);
    controllerScalingChooser.setDefaultOption("Competition/Cubic", ControllerScaling.CUBIC);

    driveTypeChooser.setDefaultOption("Competition/Semi Curvature", DriveType.SEMI_CURVATURE);
    driveTypeChooser.addOption("Competition/Reg Curvature", DriveType.REG_CURVATURE);
    driveTypeChooser.addOption("Competition/Arcade", DriveType.ARCADE);

    armControllerScalingChooser.addOption("Competition/Arm Limited Polynomic", ControllerScaling.LIMITED_POLYNOMIC);
    armControllerScalingChooser.addOption("Competition/Arm Linear", ControllerScaling.LINEAR);
    armControllerScalingChooser.addOption("Competition/Arm Squared", ControllerScaling.SQUARED);
    armControllerScalingChooser.setDefaultOption("Competition/Arm Cubic", ControllerScaling.CUBIC);

    armControlModeChooser.setDefaultOption("Position", ArmControlType.POSITION);
    armControlModeChooser.addOption("Velocity", ArmControlType.VELOCITY);
    armControlModeChooser.addOption("Cartesian", ArmControlType.CARTESIAN);
    
    SmartDashboard.putData("Competition/Autonomous Mode", autoChooser);

    SmartDashboard.putData("Competition/Driver Controls", driverControlsChooser);
    SmartDashboard.putData("Competition/Drive Controller Scaling", controllerScalingChooser);
    SmartDashboard.putData("Competition/Drive Type", driveTypeChooser);

    SmartDashboard.putData("Competition/Arm Control Mode", armControlModeChooser);
    SmartDashboard.putData("Competition/Arm Controller Scaling", armControllerScalingChooser);

    // Constants
    SmartDashboard.putNumber("Drive/Ramp Rate",Constants.DriveConstants.kRampRate);
    SmartDashboard.putNumber("Drive/Deadband",Constants.DriveConstants.kDeadband);
    SmartDashboard.putNumber("Drive/Slew Rate Limiter",Constants.DriveConstants.kSlewRateLimiter);
    SmartDashboard.putNumber("Drive/Speed Output Mod", Constants.DriveConstants.kSpeedOutputModifier);
    SmartDashboard.putNumber("Drive/Rot Output Mod", Constants.DriveConstants.kRotationOutputModifier);

    
    

}

  public void getDrivePreferences(){
    getDriverControlsChooser();
    getDriveTypeChooser();
    getControllerScalingChooser();
  }

  public void getArmPreferences(){
    getArmController();
    getArmControlModeChooser();
    getArmControllerScalingChooser();
  }

  public XboxController getDriveController() {
    return m_driveController;
  }

  public XboxController getArmController() {
    return m_armController;
  }

  public DriveControllerMode getDriverControlsChooser() {
    return driverControlsChooser.getSelected();
  }

  public DriveType getDriveTypeChooser() {
    return driveTypeChooser.getSelected();
  }

  public ControllerScaling getControllerScalingChooser() {
    return controllerScalingChooser.getSelected();
  }

  public ArmControlType getArmControlModeChooser() {
    return armControlModeChooser.getSelected();
  }

  public ControllerScaling getArmControllerScalingChooser() {
    return armControllerScalingChooser.getSelected();
  }

  public AutoCommand getAutoCommandChoice() {
    // The selected command will be run in autonomous
    return autoChooser.getSelected();
  }

}
