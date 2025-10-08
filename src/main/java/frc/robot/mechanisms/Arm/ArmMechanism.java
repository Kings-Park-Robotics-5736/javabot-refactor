package frc.robot.mechanisms.Arm;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.mechanisms.Mechanism;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;

public abstract class ArmMechanism extends Mechanism{

    protected  Limits m_limits;
    protected int updateCounter = 0;
    protected final int kUpdateInterval = 10; //update every 10 cycles
    protected double m_targetPosition = 0;

     public ArmMechanism(Limits limits, String name, PidConstants pid, FeedForwardConstants ff) {
        super(name, pid, ff,  ElevatorConstants.kDiffThreshold, ElevatorConstants.kStaleThreshold, ElevatorConstants.kStaleTolerance);
        m_limits = limits;

        toggleShowPIDTuning(false); //set to true to enable PID tuning on dashboard
    }

     @Override
    public void updatePeriodicSmartDashboard(){
        updateCounter++;
        if(updateCounter >= kUpdateInterval){
            updateCounter = 0;
            SmartDashboard.putNumber("Arm Pos Rad", getArmPositionRadians());
        }
        super.updatePeriodicSmartDashboard();
    }


    @Override
    public void periodic() {

        updatePeriodicSmartDashboard();

        if(!manualControl){
            RunArm();
        }
    }

    @Override
    public void stopMechanism() {
        setSpeed(0);
    }

    public void stopArm() {
        stopMechanism();
    }

    public Boolean isFinished(){
        return ReachedTarget(getArmPositionRadians(), m_targetPosition);
    }

    private Boolean isFinishedEarly() {
        double delta = Math.abs(getArmPositionRadians() - m_targetPosition);
        boolean isFinished =  ReachedTarget(getArmPositionRadians(), m_targetPosition);
        return isFinished || delta < Math.toRadians(10);
    }

     /******************************************************
     * Abstract Methods to be written by the specific motor elevator class
     ******************************************************/
    public abstract void setSpeed(double speed);
    public abstract double getArmPositionRadians();
    protected abstract double getArmPositionRadiansPerSecond();
    protected abstract void resetEncoder();


    protected abstract void InitMotion(double position);
    protected abstract void RunArm();


    /********************************************************
     * Commands
     ********************************************************/

    public Command RunArmToPositionCommand(double setpoint, Boolean async) {
        return new FunctionalCommand(
                () -> {InitMotion(setpoint);},
                () -> {},
                (interrupted) -> {},
                () -> {return async || isFinished();},this).withName("RunArmToPositionCommand");
    }

    public Command RunArmToPositionCommandEarlyStop(double setpoint) {
        return new FunctionalCommand(
                () -> {InitMotion(setpoint);},
                () -> {},
                (interrupted) -> {},
                () -> {return isFinishedEarly();},this).withName("RunArmToPositionCommand");
    }

    public Command RunArmManulSpeedCommand(DoubleSupplier getSpeed){
        return new FunctionalCommand(
                () -> {manualControl = true;},
                () -> {setSpeed(getSpeed.getAsDouble());},
                (interrupted) -> {stopArm();},
                () -> {return false;},
                this).withName("RunArmManulSpeedCommand");
    }

}
