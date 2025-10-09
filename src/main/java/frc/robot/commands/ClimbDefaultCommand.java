package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climb;

public class ClimbDefaultCommand extends Command{
    public ClimbDefaultCommand(Climb climb){

        addRequirements(climb);

    }
}