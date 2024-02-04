package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private final Servo RomiServo;
    private final Servo armservo;
    private final Servo TilitServo;

    public int A = 1;
    public int B = 2;
    public int X = 3;
    public int Y = 4;
    public int TriggerL = 5;
    public int TriggerR = 6;

    public Arm(){
        TilitServo = new Servo(2);
        armservo = new Servo(3);
        RomiServo = new Servo(4);
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
     
    public void setAngle(double angleDeg){
        RomiServo.setAngle(angleDeg);
    }

    public void setAngle2(double angleDeg2){
        armservo.setAngle(angleDeg2);
    }

    public void setAngle3(double angleDeg3){
        TilitServo.setAngle(angleDeg3);
    }
}
