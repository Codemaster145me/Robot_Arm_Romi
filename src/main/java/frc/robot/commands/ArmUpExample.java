package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;

public class ArmUpExample extends Command{
    @Override
    public void execute(){
        if(Constants.ArmAngle >= Constants.MaxArmUpAngle){
            Constants.ArmAngle = Constants.ArmAngle - Constants.ArmSpeed;
        }
    }
}