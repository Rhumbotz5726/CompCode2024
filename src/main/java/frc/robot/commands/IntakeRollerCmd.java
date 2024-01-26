package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class IntakeRollerCmd extends Command{
    private final frc.robot.subsystems.IntakeRollerSub roller;
    private final double speed; 
    //Trying to use a timer to run auto 
    private final Timer timer = new Timer();

    private final double time; 

    public IntakeRollerCmd(frc.robot.subsystems.IntakeRollerSub roller, double rollerTime, double speed){
        this.roller = roller;
        this.speed = speed;

        time = rollerTime;
        addRequirements(roller);
    }
    private void addRequirements(frc.robot.subsystems.IntakeRollerSub roller){

    }
    @Override 
    public void initialize(){
       timer.reset();
       timer.start();
    }
  
    @Override 
    public void execute(){
        roller.setMotor(speed);
      
    }
  
    @Override 
    public void end(boolean interrupted){
        roller.setMotor(0);
        timer.reset();

      
  
    }
  
    @Override 
    public boolean isFinished(){
       return  timer.get() >= time;
       
    }

 


}