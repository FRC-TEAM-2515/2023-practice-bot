package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.OIReporters;

public class OI {
    
    private static OI instanceOI;

    private static XboxController m_driveController;
    private static XboxController m_armController;

    private static CommandXboxController m_commandDriveController;

    private static SendableChooser<Command> autoChooser;

    private static SendableChooser<Integer> driverControlsChooser;
    private static SendableChooser<Integer> controllerScalingChooser;
    private static SendableChooser<Integer> driveTypeChooser;



  public static OI getInstance() {
      if (instanceOI == null) {
        instanceOI = new OI();
      }
      return instanceOI;
    }

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

}

public void initOI() {
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

      OI.configureButtonBindings();
      OI.configureSmartDashboard();
      OI.updateReporters();

}

public static void configureButtonBindings() {

    shouldEnableBrakes();
    shouldInvertMotors();

    m_commandDriveController.leftStick().onTrue(new SafeReset());
}

public void getDrivePreference(){
  getDriveController();
  getDriveTypeChooser();
  getControllerScalingChooser();
}


public static boolean shouldInvertMotors() {
    return m_driveController.getAButtonPressed();
  }

public static boolean shouldEnableBrakes() {
    return m_driveController.getBButton();
}

public static boolean shouldSafeReset() { 
    return m_driveController.getLeftStickButton();
}
      
public static void configureSmartDashboard() {

    // Choosers
    autoChooser.setDefaultOption("Simple Autonomous", getAutonomousCommand());

    driverControlsChooser.setDefaultOption("Left Stick", 0);
    driverControlsChooser.addOption("Trigger Acceleration", 1);

    controllerScalingChooser.setDefaultOption("Cubic", 0);
    controllerScalingChooser.addOption("Linear", 1);
    controllerScalingChooser.addOption("Squared", 2);
    controllerScalingChooser.addOption("Limited Polynomic", 3);

    driveTypeChooser.setDefaultOption("Semi Curvature", 0);
    driveTypeChooser.addOption("Reg Curvature", 1);
    driveTypeChooser.addOption("Arcade", 2);
    
    SmartDashboard.putData("Autonomous Mode", autoChooser);
    SmartDashboard.putData("Driver Controls", driverControlsChooser);
    SmartDashboard.putData("Drive Controller Scaling", controllerScalingChooser);
    SmartDashboard.putData("Drive Type", driveTypeChooser);

    // Constants
    SmartDashboard.putNumber("Ramp Rate",Constants.DriveConstants.kRampRate);
    SmartDashboard.putNumber("Deadband",Constants.DriveConstants.kDeadband);
    SmartDashboard.putNumber("Slew Rate Limiter",Constants.DriveConstants.kSlewRateLimiter);
    SmartDashboard.putNumber("Speed Output Mod", Constants.DriveConstants.kSpeedOutputModifier);
    SmartDashboard.putNumber("Rot Output Mod", Constants.DriveConstants.kRotationOutputModifier);

}

public static void updateReporters() {

  SmartDashboard.putString("DControl Mode", OIReporters.driveControllerMode);
  SmartDashboard.putNumber("LStick Speed", OIReporters.lStickSpeed );
  SmartDashboard.putNumber("TAccel Speed", OIReporters.tAccelSpeed);

  SmartDashboard.putNumber("OG Speed", OIReporters.originalSpeed );
  SmartDashboard.putNumber("OG Rotation", OIReporters.originalRotation );

  SmartDashboard.putString("Scaling", OIReporters.scalingMode);
  SmartDashboard.putString("Linear", OIReporters.linearScaled);
  SmartDashboard.putString("Squared", OIReporters.squaredScaled);
  SmartDashboard.putString("Cubic", OIReporters.cubicScaled);
  SmartDashboard.putString("Fancy", OIReporters.fancyScaled);

  SmartDashboard.putString("Drive Type Chosen", OIReporters.driveType);
  SmartDashboard.putBoolean("Semi Curvature", OIReporters.semiCurvature);
  
  SmartDashboard.putBoolean("Enabled Brakes", OIReporters.brakesEnabled);
  SmartDashboard.putNumber("Inversion Multiplier", OIReporters.inversionMult);

}


public static XboxController getDriveController() {
    return m_driveController;
  }

  public static int getDriverControlsChooser() {
    return driverControlsChooser.getSelected();
  }

  public static int getDriveTypeChooser() {
    return driveTypeChooser.getSelected();
  }

  public static int getControllerScalingChooser() {
    return controllerScalingChooser.getSelected();
  }

  public static Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoChooser.getSelected();
  }

}
