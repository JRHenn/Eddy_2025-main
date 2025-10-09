package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.MechConstants;
import com.ctre.phoenix6.hardware.TalonFX;


public class Climb extends SubsystemBase{
    
static int eieio = 0;
    public static final Command climbDefaultCommand() {
        return Commands.run(() -> eieio++);
    }

    /* - - - - - - - - - Declare Variables & Components Here - - - - - - - - - */
    private static final Joystick buttonBox = MechConstants.buttonBox;
    boolean climbyTime = false;
    boolean teleopHasStarted = Robot.teleopHasStarted;
    public TalonFX climbMotor = new TalonFX(MechConstants.climbMotorID);

    boolean climbGood = false;

    public void resetEncoder(){
        climbMotor.setPosition(0.0);
    }


    @Override
    public void periodic(){
        
        boolean teleopHasStarted = Robot.teleopHasStarted;

        if(teleopHasStarted){

            /* - - - - - - - - - Teleop Loop Code Here - - - - - - - - - */

            if(buttonBox.getRawButton(MechConstants.climbButtonID) && buttonBox.getRawButton(MechConstants.climbConfirmButtonID)){
                climbyTime = true;
            }
            
            if(climbyTime && (climbMotor.getPosition().getValueAsDouble() > MechConstants.climbDownPos) && !climbGood){
                climbMotor.set(MechConstants.climbSpeed);
            } else if(climbyTime){
                climbMotor.set(0.0);
                climbGood = true;
            } else {
                climbMotor.set(0.0);
            }

        }
    }
}