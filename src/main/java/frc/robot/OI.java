package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class OI {
    
    private static OI instanceOI;

    private static XboxController m_driveController;
    private XboxController m_armController;

    private static SendableChooser<Command> autoChooser;

    private static SendableChooser<Integer> driverControlsChooser;
    private static SendableChooser<Integer> controllerScalingChooser;
    private static SendableChooser<Integer> driveModeChooser;

    private static DriveTrain driveTrain;


  public static OI getInstance() {
      if (instanceOI == null) {
        instanceOI = new OI();
      }
      return instanceOI;
    }

    private OI() {

    // Controllers
    m_driveController = new XboxController(0);
    m_armController = new XboxController(1);

    // Command choosers
    autoChooser = new SendableChooser<>();

    // Object choosers
    driverControlsChooser = new SendableChooser<>();
    controllerScalingChooser = new SendableChooser<>();
    driveModeChooser = new SendableChooser<>();

}

public void configureButtonBindings() {
    shouldEnableBrakes();
    shouldInvertMotors();
}

public boolean shouldInvertMotors() {

    return m_driveController.getAButtonPressed();
    // boolean invertButtonTest = false;
    // if (m_driveController.getAButtonPressed() == true) {
    //     driveTrain.invertMotors(true);
    //     invertButtonTest = true;
    // }

    //SmartDashboard.putBoolean("Invert Button", invertButtonTest);
  }

  public boolean shouldEnableBrakes() {
    return m_driveController.getBButton();
    
  
  //SmartDashboard.putBoolean("Brake Button", brakeButtonTest);
}
      
public void configureSmartDashboard() {

    // Choosers
    autoChooser.setDefaultOption("Simple Autonomous", getAutonomousCommand());

    driverControlsChooser.setDefaultOption("Left Stick", 0);
    driverControlsChooser.addOption("Trigger Acceleration", 1);

    controllerScalingChooser.setDefaultOption("Cubic", 0);
    controllerScalingChooser.addOption("Linear", 1);
    controllerScalingChooser.addOption("Squared", 2);
    controllerScalingChooser.addOption("Limited Polynomic", 3);

    driveModeChooser.setDefaultOption("Semi Curvature", 0);
    driveModeChooser.addOption("Reg Curvature", 1);
    driveModeChooser.addOption("Arcade", 2);
    
    SmartDashboard.putData("Autonomous Mode", autoChooser);
    SmartDashboard.putData("Driver Controls", driverControlsChooser);
    SmartDashboard.putData("Drive Controller Scaling", controllerScalingChooser);
    SmartDashboard.putData("Drive Mode", driveModeChooser);

    // Constants
    SmartDashboard.putNumber("Ramp Rate",Constants.DriveConstants.kRampRate);
    SmartDashboard.putNumber("Deadband",Constants.DriveConstants.kDeadband);
    SmartDashboard.putNumber("Slew Rate Limiter",Constants.DriveConstants.kSlewRateLimiter);
  }

public XboxController getDriveController() {
    return m_driveController;
  }

  public int getDriverControlsChooser() {
    return driverControlsChooser.getSelected();
  }

  public int getDriveModeChooser() {
    return driveModeChooser.getSelected();
  }

  public int getControllerScalingChooser() {
    return controllerScalingChooser.getSelected();
  }

  public Command getAutonomousCommand() {
    // The selected command will be run in autonomous
    return autoChooser.getSelected();
  }

}
