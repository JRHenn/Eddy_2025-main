package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.MechConstants;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;

public class Coral extends SubsystemBase{
    
static int eieio = 0;
    public static final Command coralDefaultCommand() {
        return Commands.run(() -> eieio++);
    }

    /* - - - - - - - - - Declare Variables & Components Here - - - - - - - - - */
    private static final Joystick buttonBox = MechConstants.buttonBox;

    final PIDController pivotPID = new PIDController(MechConstants.coralPivotP, MechConstants.coralPivotI, MechConstants.coralPivotD);
    public final SparkMax pivotMotor = new SparkMax(MechConstants.coralPivotMotorID, MotorType.kBrushless);
    final SparkMax intakeMotor = new SparkMax(MechConstants.coralIntakeMotorID, MotorType.kBrushless);
    public final RelativeEncoder pivotEncoder = pivotMotor.getEncoder();

    final PIDController elevatorPID = new PIDController(MechConstants.elevatorP, MechConstants.elevatorI, MechConstants.elevatorD);
    public final TalonFX elevatorMotor = new TalonFX(MechConstants.elevatorMotorID);

    public double pivotTarget = 0.0;
    public double elevatorTarget = 0.0;

    double posOfDown = MechConstants.coralPivotInPos;

    double pivotPos = 0.0;
    double elevatorPos = 0.0;

    String elevatorAutoPos = "";


    public Command elevatorDownCommand() {
        return Commands.runOnce(() -> elevatorAutoPos = "Down");
    }

    public Command elevatorSourceCommand() {
        return Commands.runOnce(() -> elevatorAutoPos = "Source");
    }

    public Command elevatorL1Command() {
        return Commands.runOnce(() -> elevatorAutoPos = "L1");
    }

    public Command elevatorL2Command() {
        return Commands.runOnce(() -> elevatorAutoPos = "L2");
    }

    public Command elevatorL3Command() {
        return Commands.runOnce(() -> elevatorAutoPos = "L3");
    }

    public Command elevatorL4Command() {
        return Commands.runOnce(() -> elevatorAutoPos = "L4");
    }

    public Command coralIntakeCommand() {
        return Commands.runOnce(() -> intakeMotor.set(MechConstants.slowCoralIntakeSpeed));
    }

    public Command coralOutakeCommand() {
        return Commands.runOnce(() -> intakeMotor.set(MechConstants.coralOutakeSpeed));
    }

    public Command coralStopIntakeCommand() {
        return Commands.runOnce(() -> intakeMotor.set(0.0));
    }


    public void resetEncoders(){
        pivotEncoder.setPosition(0.0);
        elevatorMotor.setPosition(0.0);
    }


    @Override
    public void periodic(){
        
        boolean teleopHasStarted = Robot.teleopHasStarted;
        boolean autoHasStarted = Robot.autoHasStarted;

        pivotPos = pivotEncoder.getPosition() * 1000;

        elevatorPos = elevatorMotor.getPosition().getValueAsDouble() * 100;

        if(teleopHasStarted){

            /* - - - - - - - - - - - - - Teleop Loop Code Here - - - - - - - - - - - - - */


            // Intake & Outake
            if(buttonBox.getRawButton(MechConstants.coralIntakeButtonID)){
                intakeMotor.set(MechConstants.coralIntakeSpeed);
            } else if(buttonBox.getRawButton(MechConstants.coralOutakeButtonID)){
                intakeMotor.set(MechConstants.coralOutakeSpeed);
            } else if(buttonBox.getRawButton(MechConstants.slowCoralIntakeButtonID)){
                intakeMotor.set(MechConstants.slowCoralIntakeSpeed);
            } else {
                intakeMotor.set(0.0);
            }

            
            // Pivot Reset

            if(buttonBox.getRawButton(MechConstants.coralPivotResetButtonID)){
                pivotEncoder.setPosition(MechConstants.coralPivotResetPos / 1000);
                pivotTarget = MechConstants.coralPivotResetPos;
            }

            
            // Pivot PID
            if(pivotTarget != MechConstants.coralPivotDownPos){
                pivotMotor.set(pivotPID.calculate(pivotPos, pivotTarget) + MechConstants.coralPivotF);
            } else {
                pivotMotor.set(0.0);
            }
    

            // Elevator PID

            if(elevatorTarget != MechConstants.elevatorDownPos || elevatorPos <= MechConstants.elevatorDownPos - 500){
                elevatorMotor.set(elevatorPID.calculate(elevatorPos, elevatorTarget) + MechConstants.elevatorF);
                posOfDown = MechConstants.coralPivotInPos;
            } else {
                elevatorMotor.set(0.0);
                posOfDown = MechConstants.coralPivotDownPos;
            }

            if(elevatorTarget == MechConstants.elevatorDownPos){
                pivotTarget = posOfDown;
            }

            // Elevator and Pivot position targeter
            if(buttonBox.getRawButton(MechConstants.elevatorDownButtonID)){
                elevatorTarget = MechConstants.elevatorDownPos;
                pivotTarget = posOfDown;
            } else
            if(buttonBox.getRawButton(MechConstants.elevatorSourceButtonID)){
                elevatorTarget = MechConstants.elevatorSourcePos;
                pivotTarget = MechConstants.coralPivotInPos;
            } else
            if(buttonBox.getRawButton(MechConstants.elevatorL1ButtonID)){
                elevatorTarget = MechConstants.elevatorL1Pos;
                pivotTarget = MechConstants.coralPivotOutPos;
            } else
            if(buttonBox.getRawButton(MechConstants.elevatorL2ButtonID)){
                elevatorTarget = MechConstants.elevatorL2Pos;
                pivotTarget = MechConstants.coralPivotOutPos;
            } else
            if(buttonBox.getRawButton(MechConstants.elevatorL3ButtonID)){
                elevatorTarget = MechConstants.elevatorL3Pos;
                pivotTarget = MechConstants.coralPivotOutPos;
            } else
            if(buttonBox.getRawButton(MechConstants.elevatorL4ButtonID)){
                elevatorTarget = MechConstants.elevatorL4Pos;
                pivotTarget = MechConstants.coralPivotL4Pos;
            }

            // Pivot manual override
            if(buttonBox.getRawAxis(MechConstants.overrideJoystick_X_ID) > 0.5){
                pivotTarget -= MechConstants.coralPivotManualMargin;
            } else if(buttonBox.getRawAxis(MechConstants.overrideJoystick_X_ID) < -0.5){
                pivotTarget += MechConstants.coralPivotManualMargin;
            }

            // Elevator manual override
            if(buttonBox.getRawAxis(MechConstants.overrideJoystick_Y_ID) < -0.5){
                elevatorTarget += MechConstants.elevatorManualMargin;
            } else if(buttonBox.getRawAxis(MechConstants.overrideJoystick_Y_ID) > 0.5){
                elevatorTarget -= MechConstants.elevatorManualMargin;
            }

        } else  if(autoHasStarted){
            // Autonomous Loop Code Here

            // Pivot PID
            if(pivotTarget != MechConstants.coralPivotDownPos){
                pivotMotor.set(pivotPID.calculate(pivotPos, pivotTarget) + MechConstants.coralPivotF);
            } else {
                pivotMotor.set(0.0);
            }


            // Elevator PID

            if(elevatorTarget != MechConstants.elevatorDownPos || elevatorPos <= MechConstants.elevatorDownPos - 500){
                elevatorMotor.set(elevatorPID.calculate(elevatorPos, elevatorTarget) + MechConstants.elevatorF);
                posOfDown = MechConstants.coralPivotInPos;
            } else {
                elevatorMotor.set(0.0);
                posOfDown = MechConstants.coralPivotDownPos;
            }

            if(elevatorTarget == MechConstants.elevatorDownPos){
                pivotTarget = posOfDown;
            }


            // Elevator and Pivot position targeter
            if(elevatorAutoPos.equals("Down")){
                elevatorTarget = MechConstants.elevatorDownPos;
                pivotTarget = posOfDown;
                System.out.println("Down");
            } else
            if(elevatorAutoPos.equals("Source")){
                elevatorTarget = MechConstants.elevatorSourcePos;
                pivotTarget = MechConstants.coralPivotInPos;
                System.out.println("Source");
            } else
            if(elevatorAutoPos.equals("L1")){
                elevatorTarget = MechConstants.elevatorL1Pos;
                pivotTarget = MechConstants.coralPivotOutPos;
            } else
            if(elevatorAutoPos.equals("L2")){
                elevatorTarget = MechConstants.elevatorL2Pos;
                pivotTarget = MechConstants.coralPivotOutPos;
            } else
            if(elevatorAutoPos.equals("L3")){
                elevatorTarget = MechConstants.elevatorL3Pos;
                pivotTarget = MechConstants.coralPivotOutPos;
            } else
            if(elevatorAutoPos.equals("L4")){
                elevatorTarget = MechConstants.elevatorL4Pos;
                pivotTarget = MechConstants.coralPivotL4Pos;
                System.out.println("L4");
            }

        }
    }

}