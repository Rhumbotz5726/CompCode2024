package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PIDSub;

// This command is For controlling the PIDSub during teleop normally with no PID 
//FOr PID the PIDCMD.java command is ready all needed is for a setpoint to be called in the Robot container 


public class PIDteleopCmd extends Command {
    private final double speed;


    public PIDteleopCmd(PIDSub pidSub, double speed){//,double distance){
        this.speed = speed;
        //this.distance = distance;
        addRequirements(pidSub);
    }

    @Override
    public void initialize(){       
    }




    @Override 
    public void execute(){
    PIDSub.setMotor(speed);
          
    }

    @Override 
    public void end(boolean interrupted){
        PIDSub.setMotor(0);      
    }

    @Override
    public boolean isFinished(){
    
            return false;
        
    }
    
}
