package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class LowerShooter extends CommandBase {

    private final ShooterSubsystem shooterSubsystem;

    public LowerShooter(ShooterSubsystem shooterSubsystem){
        this.shooterSubsystem = shooterSubsystem;
    }

    @Override
    public void initialize(){
        shooterSubsystem.lowerShooter();
    }

    @Override
    public boolean isFinished(){
        return shooterSubsystem.getLowerLimitSwitch();
    }

    @Override
    public void end(boolean interrupted){
        shooterSubsystem.stopRotation();
    }

}
