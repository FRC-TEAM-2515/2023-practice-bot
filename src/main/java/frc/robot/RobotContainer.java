// RobotBuilder Version: 5.0
//
// This file was generated by RobotBuilder. It contains sections of
// code that are automatically generated and assigned by robotbuilder.
// These sections will be updated in the future when you export to
// Java from RobotBuilder. Do not put any code or make any change in
// the blocks indicating autogenerated code or it will be lost on an
// update. Deleting the comments indicating the section will prevent
// it from being updated in the future.

// ROBOTBUILDER TYPE: RobotContainer.

package frc.robot;

import frc.robot.util.OIReporters.AutoCommand;
import frc.robot.commands.*;
import frc.robot.commands.auto.AutoDriveBackwards;
//import frc.robot.commands.auto.SimpleAutonomous;
import frc.robot.commands.auto.AutoDriveBackwards;
import frc.robot.subsystems.*;
import frc.robot.util.OIReporters;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public class RobotContainer {

  // The robot's subsystems
  // public final DriveTrain m_driveTrain = new DriveTrain();

  public static RobotContainer robotContainer = new RobotContainer();

  private DriveTrain driveTrain;
  private Arm arm;
  private Vision vision;
  // private Vision vision;
  // private Claw claw;
  // private Wrist wrist;
  // private Arm arm;
  // private Turret turret;

  private OI oi;
  private OIReporters oiReporters;

  private DriveCommand driveCommand;
  private ArmCommand armCommand;

  private XboxController driveController;

  private Command autonomous;

  public static RobotContainer getInstance() {
    if (robotContainer == null) {
      robotContainer = new RobotContainer();
    }
    return robotContainer;
  }

  public RobotContainer() {
    initSubsystems();
    configOI();

  }

  private void initSubsystems() {

    driveTrain = new DriveTrain();
    arm = new Arm();
    vision = new Vision();
  }

  // OI
  private void configOI() {
    oi = new OI();
    oi.initOI();
    oiReporters = new OIReporters();
    oiReporters.updateOIReporters();

  }

  public OI getOI() {
    return oi;
  }

  public OIReporters getOIReporters() {
    return oiReporters;
  }

  // Drive
  public void initDefaultDrive() {
    driveCommand = new DriveCommand(driveTrain, oi.getDriveController());
    driveTrain.setDefaultCommand(driveCommand);
  }

  public DriveTrain getDriveTrain() {
    return driveTrain;
  }

  public void initArmDefault() {
    armCommand = new ArmCommand(arm, oi.getArmController());
    arm.setDefaultCommand(armCommand);
  }

  public Arm getArm() {
    return arm;
  }

  // Misc Commands
  public void safeReset() {
    driveTrain.stopMotors();
    driveTrain.resetEncoders();
    SmartDashboard.putBoolean("Safe Reset", true);
  }

  public Command getAutoCommand() {
    if (robotContainer.getOI().getAutoCommandChoice() == OIReporters.AutoCommand.SIMPLE){
    autonomous = new AutoDriveBackwards(driveTrain, 3.5);
      //autonomous = new SimpleAutonomous(driveTrain, 0); 
 }
 return autonomous;
}
}
