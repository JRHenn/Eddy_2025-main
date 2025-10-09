// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.constants.MechConstants;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;

public class Robot extends TimedRobot {
  public static boolean teleopHasStarted = false;
  public static boolean autoHasStarted = false;

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  UsbCamera myCamera;

  public static double driveSpeedY = 0.0;

  public static double driveSpeedX = 0.0;

  private Joystick driveStick = MechConstants.driveStick;


  public Robot() {
    m_robotContainer = new RobotContainer();
    
    myCamera = CameraServer.startAutomaticCapture();
    myCamera.setResolution(360, 240);
  }

  int chooserClock = 0; // Clock for printing out the chosen autonomous routine
  String oldAutoChosen = "waiting...";
  String newAutoChosen = "waiting...";


  @Override
  public void robotPeriodic() {

    CommandScheduler.getInstance().run(); 

    if(driveStick.getRawAxis(0) >= 0.0){
      driveSpeedY = Math.pow(driveStick.getRawAxis(1), MechConstants.driveExponent);
    } else {
      driveSpeedY = -Math.abs(Math.pow(driveStick.getRawAxis(1), MechConstants.driveExponent));
    }

    if(driveStick.getRawAxis(1) >= 0.0){
      driveSpeedX = Math.pow(driveStick.getRawAxis(0), MechConstants.driveExponent);
    } else {
      driveSpeedX = -Math.abs(Math.pow(driveStick.getRawAxis(0), MechConstants.driveExponent));
    }


    newAutoChosen = RobotContainer.autoChooser.getSelected().getName();

    if (!newAutoChosen.equals(oldAutoChosen) || chooserClock % 1500 == 0) {
      System.out.println("Auto Chosen: - " + newAutoChosen);    // Print out chosen auto if auto changes
    }                                                           // or every 30 seconds

    oldAutoChosen = RobotContainer.autoChooser.getSelected().getName();

    chooserClock++;


    SmartDashboard.putNumber("Elevator Motor Power", m_robotContainer.myCoral.elevatorMotor.get());
    SmartDashboard.putNumber("Elevator Position", m_robotContainer.myCoral.elevatorMotor.getPosition().getValueAsDouble() * 100);
    SmartDashboard.putNumber("Elevator Target", m_robotContainer.myCoral.elevatorTarget);
    SmartDashboard.putNumber("Pivot Motor Power", m_robotContainer.myCoral.pivotMotor.get());
    SmartDashboard.putNumber("Pivot Position", m_robotContainer.myCoral.pivotEncoder.getPosition() * 1000);
    SmartDashboard.putNumber("Pivot Target", m_robotContainer.myCoral.pivotTarget);
    SmartDashboard.putNumber("Climb Motor Power", m_robotContainer.myClimb.climbMotor.get());
    SmartDashboard.putNumber("Climb Position", m_robotContainer.myClimb.climbMotor.getPosition().getValueAsDouble());

  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }

    autoHasStarted = true;
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    teleopHasStarted = true;
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  @Override
  public void simulationPeriodic() {}
}