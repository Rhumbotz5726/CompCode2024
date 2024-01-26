package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ServoCmd extends Command{
    private final frc.robot.subsystems.Servo servo;
    
    private final int angle; 

    public ServoCmd(frc.robot.subsystems.Servo servo, int angle){
        this.servo = servo;
        this.angle = angle;
        addRequirements(servo);
    }
    private void addRequirements(frc.robot.subsystems.Servo servo){

    }
    @Override 
    public void initialize(){
        //System.out.println("Servo started! ");
    }
  
    @Override 
    public void execute(){
        servo.setServo(angle);
     
    }
  
    @Override 
    public void end(boolean interrupted){
       // servo.setServo(0);
      
       
      
  
    }
  
    @Override 
    public boolean isFinished(){
        return false;
    }


}