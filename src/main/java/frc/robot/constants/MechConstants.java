package frc.robot.constants;

import edu.wpi.first.wpilibj.Joystick;

public class MechConstants {
    
    /* - - - - - - - - - Joystick IDs - - - - - - - - - */
    public static final Joystick buttonBox = new Joystick(2);
    public static final Joystick driveStick = new Joystick(0);
    public static final Joystick steerStick = new Joystick(1);
    
    /* - - - - - - - - - Button IDs - - - - - - - - - */    //        COMMENTS ARE FOR JEFF'S BUTTON BOX
    public static final int coralIntakeButtonID = 8;     // 7
    public static final int slowCoralIntakeButtonID = 10;     // 99
    public static final int coralOutakeButtonID = 7;     // 8
    public static final int coralPivotResetButtonID = 11;     // 99

    public static final int elevatorDownButtonID = 6;    // 99
    public static final int elevatorSourceButtonID = 5;    // 1
    public static final int elevatorL1ButtonID = 2;    // 99
    public static final int elevatorL2ButtonID = 2;    // 2
    public static final int elevatorL3ButtonID = 3;    // 3
    public static final int elevatorL4ButtonID = 4;    // 4

    public static final int overrideJoystick_X_ID = 1;    // 0
    public static final int overrideJoystick_Y_ID = 0;    // 1

    public static final int climbButtonID = 99;    // 99
    public static final int climbConfirmButtonID = 9;    // 99

    public static final int autoAlignButton = 12;

    /* - - - - - - - - - Motor IDs - - - - - - - - - */
    public static final int elevatorMotorID = 13;
    public static final int coralIntakeMotorID = 16;
    public static final int coralPivotMotorID = 15;

    public static final int climbMotorID = 14;

    /* - - - - - - - - - Motor Speeds - - - - - - - - - */
    public static final double coralIntakeSpeed = 0.35;
    public static final double slowCoralIntakeSpeed = 0.15;
    public static final double coralOutakeSpeed = -1.0;

    public static final double climbSpeed = -0.2;

    public static final double driveExponent = 3.0; // driveSpeed = driveStick to the power of this

    /* - - - - - - - - - Motor PID Constants - - - - - - - - - */
    public static final double elevatorP = 0.0004;
    public static final double elevatorI = 0.0;
    public static final double elevatorD = 0.0;
    public static final double elevatorF = -0.03;

    public static final double coralPivotP = 0.00003;
    public static final double coralPivotI = 0.0;
    public static final double coralPivotD = 0.0;
    public static final double coralPivotF = -0.015;

    /* - - - - - - - - - Motor Position Constants - - - - - - - - - */
    public static final double climbDownPos = -35;

    public static final double coralPivotInPos = -16500;
    public static final double coralPivotOutPos = -8500;
    public static final double coralPivotL4Pos = -10000;
    public static final double coralPivotDownPos = -2500.0;
    public static final double coralPivotResetPos = -19500;
    public static final double coralPivotManualMargin = 250;

    public static final double elevatorDownPos = -300;
    public static final double elevatorSourcePos = -350;
    public static final double elevatorL1Pos = -1700;
    public static final double elevatorL2Pos = -1700;
    public static final double elevatorL3Pos = -4600;
    public static final double elevatorL4Pos = -7825;
    public static final double elevatorManualMargin = 50;

}