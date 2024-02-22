// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.ArcadeDrive;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.OnBoardIO;
import frc.robot.subsystems.OnBoardIO.ChannelMode;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

  // Assumes a gamepad plugged into channel 0
  public static Joystick m_controller = new Joystick(Constants.Driverport);

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

<<<<<<< HEAD
=======
  int ang = 180;
  GenericEntry entry = null;
  //GenericEntry entry2 = null;
  


  public void Open(){
    if (ang >= 10){
      ang = ang - 2;
    }
    //Shuffleboard.getTab("Arm").add("Angle", ang); simple code to add a number to the shuffleboard
    // Shuffleboard.update();

  this.entry.setDouble(ang);
    // entry.setDouble(ang);
  }

  public void Close(){
    if (ang <= 180){
      ang = ang + 2;
      //System.out.println(ang + "\n");
    }
  }

  double ang2 = 180;

  public void ArmUp(){
    if(ang2 >= 30){
      ang2 = ang2 - 5;
    }
    //close state is 180, when do open, we decrease the number  
    //open state is 30, when do close, we increase the number 
  }

  public void ArmDown(){
    if(ang2 <= 180){
      ang2 = ang2 + 5;
    }
    //close state is 180, when do open, we decrease the number  
    //open state is 30, when do close, we increase the number 
  }

>>>>>>> parent of ca08ec3 (arm and grabber working)
  private void configureButtonBindings() {
    // Default command is arcade drive. This will run unless another command
    // is scheduled over it.
    m_drivetrain.setDefaultCommand(getArcadeDriveCommand());

    // Example of how to use the onboard IO
    Trigger onboardButtonA = new Trigger(m_onboardIO::getButtonAPressed);
    onboardButtonA
        .onTrue(new PrintCommand("Button A Pressed"))
        .onFalse(new PrintCommand("Button A Released"));
<<<<<<< HEAD
=======

    //grabber
    //swevo 1
    ShuffleboardTab tab = Shuffleboard.getTab("graber1");
    GenericEntry entry = tab.add("graber1", ang).getEntry();
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
    //ShuffleboardTab tab2 = Shuffleboard.getTab("arm");
    //GenericEntry entry2 = tab.add("arm", ang).getEntry();
    //this.entry = entry2;
    JoystickButton joystickYButton = new JoystickButton(m_controller, m_arm.B);
      joystickYButton.whileTrue(new RepeatCommand(new InstantCommand(() -> ArmUp())));
      joystickYButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle2(ang2), m_arm)));

    JoystickButton joystickBButton = new JoystickButton(m_controller, m_arm.Y);
      joystickBButton.whileTrue(new RepeatCommand(new InstantCommand(() -> ArmDown())));
      joystickBButton.whileTrue(new RepeatCommand(new InstantCommand(() -> m_arm.setAngle2(ang2), m_arm)));
>>>>>>> parent of ca08ec3 (arm and grabber working)
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
