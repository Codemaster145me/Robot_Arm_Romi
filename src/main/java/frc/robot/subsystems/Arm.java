package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase{
    private final Servo RomiServo;
    private final Servo armservo;

    public int A = 1;
    public int B = 2;
    public int X = 3;
    public int Y = 4;

    public Arm(){
        RomiServo = new Servo(4);
        armservo = new Servo(5);
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

    public void setAngle2(double angleDeg){
        armservo.setAngle(angleDeg);
    }
}
