// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ArcadeDrive;
import frc.robot.commands.AutonomousDistance;
import frc.robot.commands.AutonomousTime;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Drivetrain m_drivetrain = new Drivetrain();
  private final OnBoardIO m_onboardIO = new OnBoardIO(ChannelMode.INPUT, ChannelMode.INPUT);
  private final Arm m_arm = new Arm();

  // Assumes a gamepad plugged into channel 0
  private final Joystick m_controller = new Joystick(0);

  // Create SmartDashboard chooser for autonomous routines
  private final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // NOTE: The I/O pin functionality of the 5 exposed I/O pins depends on the hardware "overlay"
  // that is specified when launching the wpilib-ws server on the Romi raspberry pi.
  // By default, the following are available (listed in order from inside of the board to outside):
  // - DIO 8 (mapped to Arduino pin 11, closest to the inside of the board)
  // - Analog In 0 (mapped to Analog Channel 6 / Arduino Pin 4)
  // - Analog In 1 (mapped to Analog Channel 2 / Arduino Pin 20)
  // - PWM 2 (mapped to Arduino Pin 21)
  // - PWM 3 (mapped to Arduino Pin 22)
  //
  // Your subsystem configuration should take the overlays into account

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

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

  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));

    //grabber
    //swevo 1
    ShuffleboardTab tab = Shuffleboard.getTab("Arm");
    GenericEntry entry = tab.add("Arm", ang).getEntry();
    this.entry = entry; 
    JoystickButton joystickAButton = new JoystickButton(m_controller, m_arm.A);
      joystickAButton.whileTrue(new RepeatCommand(new InstantCommand(() -> Open())));
      joystickAButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle(ang), m_arm)));
      // joystickAButton.whileTrue(new RepeatCommand(new InstantCommand(() -> System.out.println(ang))));


    JoystickButton joystickXButton = new JoystickButton(m_controller, m_arm.X);
      joystickXButton.whileTrue(new RepeatCommand(new InstantCommand(() -> Close())));
      joystickXButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle(ang), m_arm)));

    //grab
    //swevo 2
    ShuffleboardTab tab2 = Shuffleboard.getTab("Graber");
    GenericEntry entry2 = tab2.add("Graber", ang2).getEntry();
    this.entry = entry2;
    JoystickButton joystickYButton = new JoystickButton(m_controller, m_arm.B);
      joystickYButton.whileTrue(new RepeatCommand(new InstantCommand(() -> ArmUp())));
      joystickYButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle2(ang2), m_arm)));

    JoystickButton joystickBButton = new JoystickButton(m_controller, m_arm.Y);
      joystickBButton.whileTrue(new RepeatCommand(new InstantCommand(() -> ArmDown())));
      joystickBButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle2(ang2), m_arm)));

    //tilit
    //swevo 3
    ShuffleboardTab tab3 = Shuffleboard.getTab("Tilt");
    GenericEntry entry3 = tab3.add("Tilt", ang3).getEntry();
    this.entry = entry3;
    JoystickButton joystickTriggerLButton = new JoystickButton(m_controller, m_arm.TriggerL);
      joystickTriggerLButton.whileTrue(new RepeatCommand(new InstantCommand(() -> TilitDown())));
      joystickTriggerLButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle3(ang3), m_arm)));

    JoystickButton joystickTriggerRButton = new JoystickButton(m_controller, m_arm.TriggerR);
      joystickTriggerRButton.whileTrue(new RepeatCommand(new InstantCommand(() -> TilitUp())));
      joystickTriggerRButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle3(ang3), m_arm)));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */

  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }

  /**
   * Use this to pass the teleop command to the main {@link Robot} class.
   *
   * @return the command to run in teleop
   */
  public Command getArcadeDriveCommand() {
    return new ArcadeDrive(
        m_drivetrain, () -> -m_controller.getRawAxis(1), () -> -m_controller.getRawAxis(2));
  }
}
