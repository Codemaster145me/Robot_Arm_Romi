package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ArmSubsystem extends SubsystemBase{
    private static Servo RomiServo;
    private static Servo armservo;
    private static Servo TilitServo;

    public ArmSubsystem(){
        TilitServo = new Servo(Constants.TiltPort);
        armservo = new Servo(Constants.ArmPort);
        RomiServo = new Servo(Constants.GrabPort);
    }

    @Override
    public void periodic(){
        // This method will be called once per scheduler run
    }

    /**
     * Set the current angle of the arm (0 - 180 degrees)
     * 
     * @param angleDeg Desired arm angle in degrees
     */
     
    public static void setAngle(double angleDeg){
        RomiServo.setAngle(angleDeg);
    }

    public static void setAngle2(double angleDeg2){
        armservo.setAngle(angleDeg2);
    }

    public static void setAngle3(double angleDeg3){
        TilitServo.setAngle(angleDeg3);
    }

}
