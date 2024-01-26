package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

public class ServoLCmd extends Command{
    private final frc.robot.subsystems.ServoL servo2;
    private final int angle; 

    public ServoLCmd(frc.robot.subsystems.ServoL servo2, int angle){
        this.servo2 = servo2;
        this.angle = angle;
        addRequirements(servo2);
    }
    private void addRequirements(frc.robot.subsystems.ServoL servo2){

    }
    @Override 
    public void initialize(){
        //System.out.println("Servo started! ");
    }
  
    @Override 
    public void execute(){
        servo2.setServo(angle);
     
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