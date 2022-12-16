package frc.robot.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ColorSubsystem;

public class RetractSpinner extends CommandBase {
    private final ColorSubsystem colorSubsystem;

    public RetractSpinner(ColorSubsystem colorSubsystem){
        this.colorSubsystem = colorSubsystem;
    }

    @Override
    public void initialize(){
        colorSubsystem.lowerArm();
    }

    @Override
    public boolean isFinished(){
        return colorSubsystem.getBottomLimitSwitch();
    }

    @Override
    public void end(boolean interrupted){
        colorSubsystem.stopArm();
    }


}
