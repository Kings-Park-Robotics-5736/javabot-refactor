package frc.robot.subsystems.mechanisms.Wheel;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.mechanisms.Mechanism;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

public abstract class WheelMechanism extends Mechanism {

    protected int updateCounter = 0;
    protected final int kUpdateInterval = 10; //update every 10 cycles
    protected double m_targetSpeedRPS = 0;
    /*
     * ALERT!!!!!
     * The CTRE devices report RPS. The REV devices report RPM. MUST convert appropriately.
     */


    public WheelMechanism(String name, PidConstants pid, FeedForwardConstants ff, double diffThreshold, int staleThreshold, double staleTolerance) {
        super(name, pid, ff, diffThreshold, staleThreshold, staleTolerance);
        toggleShowPIDTuning(false); //set to true to enable PID tuning on dashboard
    }


    @Override
    public void updatePeriodicSmartDashboard(){
        updateCounter++;
        if(updateCounter >= kUpdateInterval){
            updateCounter = 0;
            SmartDashboard.putNumber("Wheel Speed RPM", getWheelSpeedRPS() * 60);
        }
        super.updatePeriodicSmartDashboard();
    }


    @Override
    public void periodic() {

        updatePeriodicSmartDashboard();

        if(!manualControl){
            RunWheel();
        }
    }

    @Override
    public void stopMechanism() {
        setSpeed(0);
    }

    public Boolean isFinished(){
        return ReachedTarget(getWheelSpeedRPS(), m_targetSpeedRPS);
    }

    private Boolean isFinishedEarly() {
        double delta = Math.abs(getWheelSpeedRPS() - m_targetSpeedRPS);
        boolean isFinished =  ReachedTarget(getWheelSpeedRPS(), m_targetSpeedRPS);
        return isFinished || delta < 100; //100 rps tolerance for early stop
    }

    /**
     * Initialize motion to setpoint, target is in RPS
     * @param setpoint - target speed in RPS
     */
    protected void InitMotion(double setpoint) {
        m_targetSpeedRPS = setpoint;
        manualControl = false;
        staleCounter = 0; //new target, reset stale counter
        SmartDashboard.putNumber("Wheel Setpoint RPM " + m_name, m_targetSpeedRPS * 60);
    }

     /******************************************************
     * Abstract Methods to be written by the specific motor elevator class
     ******************************************************/
    public abstract void setSpeed(double speed);
    public abstract double getWheelSpeedRPS();
    protected abstract void RunWheel();


     /********************************************************
     * Commands
     ********************************************************/

    public Command RunWheelToSpeedCommand(double setpointRPS, Boolean async) {
        return new FunctionalCommand(
                () -> {InitMotion(setpointRPS);},
                () -> {},
                (interrupted) -> {},
                () -> {return async || isFinished();},this).withName("RunWheelToSpeedCommand");
    }

    public Command RunArmToPositionCommandEarlyStop(double setpointRPS) {
        return new FunctionalCommand(
                () -> {InitMotion(setpointRPS);},
                () -> {},
                (interrupted) -> {},
                () -> {return isFinishedEarly();},this).withName("RunWheelToSpeedCommand");
    }

    public Command RunArmManulSpeedCommand(DoubleSupplier getSpeed){
        return new FunctionalCommand(
                () -> {manualControl = true;},
                () -> {setSpeed(getSpeed.getAsDouble());},
                (interrupted) -> {stopMechanism();},
                () -> {return false;},
                this).withName("RunWheelManulSpeedCommand");
    }

}
