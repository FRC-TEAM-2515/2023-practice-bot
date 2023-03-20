package frc.robot;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior; 
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.OIReporters;
import frc.robot.RobotMath;
import frc.robot.commands.auto.AutoDriveToCharge;
import frc.robot.commands.auto.Autonomous;

public class OI {
    
    private static OI instanceOI;
    private OIReporters oiReporters;
    private XboxController m_driveController;
    private XboxController m_armController;

    private CommandXboxController m_commandDriveController;

    private SendableChooser<Command> autoChooser;

    private SendableChooser<Integer> driverControlsChooser;
    private SendableChooser<Integer> controllerScalingChooser;
    private SendableChooser<Integer> driveTypeChooser;

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
    //autoChooser.addOption("Autonomous Command", new AutoDriveToCharge());
    //autoChooser.addOption("Autonomous Command", new Autonomous());
    autoChooser.setDefaultOption("Simple Autonomous", getAutonomousCommand());
               // m_chooser.setDefaultOption("Complex Auto", new complexAuto( */

    driverControlsChooser.setDefaultOption("Left Stick", 0);
    driverControlsChooser.addOption("Trigger Acceleration", 1);
    
    controllerScalingChooser.setDefaultOption("Limited Polynomic", 0);
    controllerScalingChooser.addOption("Linear", 1);
    controllerScalingChooser.addOption("Squared", 2);
    controllerScalingChooser.addOption("Cubic", 3);

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

  public int getDriverControlsChooser() {
    return driverControlsChooser.getSelected();
  }

  public int getDriveTypeChooser() {
    return driveTypeChooser.getSelected();
  }

  public int getControllerScalingChooser() {
    return controllerScalingChooser.getSelected();
  }

  public CommandBase getAutonomousCommand() {
    // The selected command will be run in autonomous
    if (autoChooser.getSelected() == new Autonomous()) {

    };
      //return new Autonomous();
    return autoChooser.getSelected();
    //return new Autonomous();
  }

}
