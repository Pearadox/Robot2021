package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.TransportSystem;


public class HopperInCmd extends CommandBase {
    private TransportSystem balltower;

    public HopperInCmd(TransportSystem tower) {
        // each subsystem used by the command must be passed into the addRequirements() method (which takes a vararg of Subsystem)
        balltower = tower;
        addRequirements(balltower);
    }
    //this is a test
    /**
     * The initial subroutine of a command.  Called once when the command is initially scheduled.
     */
    @Override
    public void initialize() {

    }

    /**
     * The main body of a command.  Called repeatedly while the command is scheduled.
     * (That is, it is called repeatedly until {@link #isFinished()}) returns true.)
     */
    @Override
    public void execute() {
        if(balltower.getBallCounter() < balltower.getMaxBallCounter()) {
            balltower.HopperIn();
        }else{
            balltower.HopperStop();
        }
    }

    @Override
    public boolean isFinished() {
        // TODO: Make this return true when this Command no longer needs to run execute()
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        balltower.HopperStop();

    }
}
