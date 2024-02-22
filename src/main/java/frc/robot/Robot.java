// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.commands.ArmUp;
import frc.robot.commands.ArmDown;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;
  private final ArmSubsystem m_arm = new ArmSubsystem();
  private final ArmUp armup = new ArmUp();
  private final ArmDown armdown = new ArmDown();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    // Get selected routine from the SmartDashboard
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  double ang = 180;
  double ang2 = 180;
  double ang3 = 100;
  GenericEntry entry = null;
  GenericEntry entry2 = null;
  GenericEntry entry3 = null;


  public void Open(){
    if (ang >= 10){
      ang = ang - 2;
    }

  this.entry.setDouble(ang);
    // entry.setDouble(ang);
  }

  public void Close(){
    if (ang <= 180){
      ang = ang + 2;
    }
  }

  public void ArmUp(){
    if(ang2 >= 8){
      ang2 = ang2 - 5;
    }
  }

  public void ArmDown(){
    if(ang2 <= 100){
      ang2 = ang2 + 5;
    }
  }

  public void TilitDown(){
    if(ang3 >= 8){
      ang3 = ang3 - 2;
    }
  }

  public void TilitUp(){
    if(ang3 <= 180){
      ang3 = ang3 + 2;
    }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running which will
    // use the default command which is ArcadeDrive. If you want the autonomous
    // to continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    //grabber
    //swevo 1
    /* 
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    GenericEntry entry = tab.add("Arm", ang).getEntry();
    this.entry = entry; */

    JoystickButton joystickAButton = new JoystickButton(RobotContainer.m_controller, Constants.A);
      joystickAButton.whileTrue(armup); // ??
      //joystickAButton.whileTrue(new RepeatCommand(new InstantCommand(() -> 
      //Shuffleboard.getTab("Arm").add("Sendable Title", ang))));
      // joystickAButton.whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println(ang))));

    JoystickButton joystickXButton = new JoystickButton(RobotContainer.m_controller, Constants.X);
      joystickXButton.whileTrue(armdown);
      //joystickXButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle(ang), m_arm)));

    //grab
    //swevo 2
    /* 
    ShuffleboardTab tab2 = Shuffleboard.getTab("Graber");
    GenericEntry entry2 = tab2.add("Graber", ang2).getEntry();
    this.entry = entry2;
    JoystickButton joystickYButton = new JoystickButton(RobotContainer.m_controller, Constants.B);
      joystickYButton.whileTrue(new RepeatCommand(new InstantCommand(() -> ArmUp())));
      joystickYButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle2(ang2), m_arm)));

    JoystickButton joystickBButton = new JoystickButton(RobotContainer.m_controller, Constants.Y);
      joystickBButton.whileTrue(new RepeatCommand(new InstantCommand(() -> ArmDown())));
      joystickBButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle2(ang2), m_arm)));
    */ 
    //tilit
    //swevo 3
    /* 
    ShuffleboardTab tab3 = Shuffleboard.getTab("Tilt");
    GenericEntry entry3 = tab3.add("Tilt", ang3).getEntry();
    this.entry = entry3;
    JoystickButton joystickTriggerLButton = new JoystickButton(RobotContainer.m_controller, Constants.TriggerL);
      joystickTriggerLButton.whileTrue(new RepeatCommand(new InstantCommand(() -> TilitDown())));
      joystickTriggerLButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle3(ang3), m_arm)));

    JoystickButton joystickTriggerRButton = new JoystickButton(RobotContainer.m_controller, Constants.TriggerR);
      joystickTriggerRButton.whileTrue(new RepeatCommand(new InstantCommand(() -> TilitUp())));
      joystickTriggerRButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle3(ang3), m_arm)));
    */
  }
  

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
}
