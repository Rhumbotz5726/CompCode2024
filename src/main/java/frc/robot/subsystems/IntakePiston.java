package frc.robot.subsystems;
// Only needed when the DOuble solenoid is being used 
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
//import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** A hatch mechanism actuated by a single {@link DoubleSolenoid}. */
public class IntakePiston extends SubsystemBase {

    private final Solenoid solenoidR = new Solenoid(PneumaticsModuleType.CTREPCM,1);
    private final Solenoid solenoidL = new Solenoid(PneumaticsModuleType.CTREPCM,2);

    // This is incase that double solenoid is needed just comment out the single solenoid and then it is ready to go 

 // private final DoubleSolenoid m_hatchSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM,1,2);
 // private final DoubleSolenoid solenoidR = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 3, 4);

// The Intake will be in when the piston is extended 

public void intakeOut(){
    solenoidL.set(false);
    solenoidR.set(false);
}

public void intakeIn(){
    solenoidL.set(true);
    solenoidR.set(true);
}

/* 
// THis is for the double Solenoid 
  //Intake comes out 
  public void grabHatch() {
    m_hatchSolenoid.set(kForward);
  }

  /** Releases the hatch. 
  public void releaseHatch() {
    m_hatchSolenoid.set(kReverse);
  }*/

}