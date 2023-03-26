package frc.robot;

import frc.robot.Constants.ArmControlType;
import frc.robot.Constants.ControllerScaling;
import frc.robot.Constants.DriveControllerMode;
import frc.robot.Constants.DriveType;
import frc.robot.commands.*;
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

public class OI {
    
    private static OI instanceOI;
    private OIReporters oiReporters;
    private XboxController m_driveController;
    private XboxController m_armController;

    private CommandXboxController m_commandDriveController;
    private Trigger resetButton;
    private JoystickButton brakeButton;

    private SendableChooser<Command> autoChooser;

    private SendableChooser<Enum> driverControlsChooser;
    private SendableChooser<Enum> controllerScalingChooser;
    private SendableChooser<Enum> driveTypeChooser;

    private SendableChooser<Enum> armControlModeChooser;
    private SendableChooser<Enum> armControllerScalingChooser;

  // public static OI getInstance() {
  //     if (instanceOI == null) {
  //       instanceOI = new OI();
  //     }
  //     return instanceOI;
  //   }

    public OI() {

    // Controllers
    m_driveController = new XboxController(0);
    //m_commandDriveController = new CommandXboxController(0);
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

}
public void initOI() {
    configureButtonBindings();
    configureSmartDashboard();
    getDrivePreferences();
}

private void configureButtonBindings() {

    shouldEnableBrakes();
    shouldInvertMotors();

    m_commandDriveController.leftStick().onTrue(new SafeReset());
    
  
}

public boolean shouldInvertMotors() {
    return m_driveController.getAButtonPressed();
  }

public boolean shouldEnableBrakes() {
    return m_driveController.getBButton();
}


public void configureSmartDashboard() {

    // Choosers
    autoChooser.setDefaultOption("Simple Autonomous", getAutonomousCommand());

    driverControlsChooser.setDefaultOption("Left Stick", DriveControllerMode.LEFT_STICK);
    driverControlsChooser.addOption("Trigger Acceleration", DriveControllerMode.TRIGGER_ACCEL);
    
    controllerScalingChooser.setDefaultOption("Limited Polynomic", ControllerScaling.LIMITED_POLYNOMIC);
    controllerScalingChooser.addOption("Linear", ControllerScaling.LINEAR);
    controllerScalingChooser.addOption("Squared", ControllerScaling.SQUARED);
    controllerScalingChooser.addOption("Cubic", ControllerScaling.CUBIC);

    driveTypeChooser.setDefaultOption("Semi Curvature", DriveType.SEMI_CURVATURE);
    driveTypeChooser.addOption("Reg Curvature", DriveType.REG_CURVATURE);
    driveTypeChooser.addOption("Arcade", DriveType.ARCADE);

    armControllerScalingChooser.addOption("Limited Polynomic", ControllerScaling.LIMITED_POLYNOMIC);
    armControllerScalingChooser.setDefaultOption("Linear", ControllerScaling.LINEAR);
    armControllerScalingChooser.addOption("Squared", ControllerScaling.SQUARED);
    armControllerScalingChooser.addOption("Cubic", ControllerScaling.CUBIC);

    armControlModeChooser.setDefaultOption("Position", ArmControlType.POSITION);
    armControlModeChooser.setDefaultOption("Velocity", ArmControlType.VELOCITY);
    armControlModeChooser.setDefaultOption("Cartesian", ArmControlType.CARTESIAN);
    
    SmartDashboard.putData("Autonomous Mode", autoChooser);
    SmartDashboard.putData("Driver Controls", driverControlsChooser);
    SmartDashboard.putData("Drive Controller Scaling", controllerScalingChooser);
    SmartDashboard.putData("Drive Type", driveTypeChooser);

    SmartDashboard.putData("Arm Control Mode", armControlModeChooser);
    SmartDashboard.putData("Arm Controller Scaling", armControllerScalingChooser);

    // Constants
    SmartDashboard.putNumber("Ramp Rate",Constants.DriveConstants.kRampRate);
    SmartDashboard.putNumber("Deadband",Constants.DriveConstants.kDeadband);
    SmartDashboard.putNumber("Slew Rate Limiter",Constants.DriveConstants.kSlewRateLimiter);
    SmartDashboard.putNumber("Speed Output Mod", Constants.DriveConstants.kSpeedOutputModifier);
    SmartDashboard.putNumber("Rot Output Mod", Constants.DriveConstants.kRotationOutputModifier);

}

// public void updateReporters() {

//   SmartDashboard.putString("DControl Mode", OIReporters.driveControllerMode);
//   SmartDashboard.putNumber("LStick Speed", OIReporters.lStickSpeed );
//   SmartDashboard.putNumber("TAccel Speed", OIReporters.tAccelSpeed);

//   SmartDashboard.putNumber("OG Speed", OIReporters.originalSpeed );
//   SmartDashboard.putNumber("OG Rotation", OIReporters.originalRotation );

//   SmartDashboard.putString("Scaling", OIReporters.scalingMode);
//   SmartDashboard.putString("Linear", OIReporters.linearScaled);
//   SmartDashboard.putString("Squared", OIReporters.squaredScaled);
//   SmartDashboard.putString("Cubic", OIReporters.cubicScaled);
//   SmartDashboard.putString("Fancy", OIReporters.fancyScaled);

//   SmartDashboard.putString("Drive Type Chosen", OIReporters.driveType);
//   SmartDashboard.putBoolean("Semi Curvature", OIReporters.semiCurvature);
  
//   SmartDashboard.putBoolean("Enabled Brakes", OIReporters.brakesEnabled);
//   SmartDashboard.putNumber("Inversion Multiplier", OIReporters.inversionMult);

// }

  public void getDrivePreferences(){
    getDriveController();
    getDriveTypeChooser();
    getControllerScalingChooser();
  }

  public XboxController getDriveController() {
    return m_driveController;
  }

  public XboxController getArmController() {
    return m_armController;
  }

  public Enum getDriverControlsChooser() {
    return driverControlsChooser.getSelected();
  }

  public Enum getDriveTypeChooser() {
    return driveTypeChooser.getSelected();
  }

  public Enum getControllerScalingChooser() {
    return controllerScalingChooser.getSelected();
  }

  public Enum getArmControlModeChooser() {
    return armControlModeChooser.getSelected();
  }

  public Enum getArmControllerScalingChooser() {
    return armControllerScalingChooser.getSelected();
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoChooser.getSelected();
  }

}
