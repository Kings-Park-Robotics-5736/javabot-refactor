package frc.robot.subsystems.mechanisms.Elevator;

import frc.robot.utils.Types.Limits;
import frc.robot.utils.Types.PidConstants;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import frc.robot.subsystems.mechanisms.Mechanism;
import frc.robot.utils.Types.FeedForwardConstants;

public abstract class ElevatorMechanism extends Mechanism  {

    protected  Limits m_limits;
    protected int updateCounter = 0;
    protected final int kUpdateInterval = 10; //update every 10 cycles
    protected final int kLimitBuffer = 3;
    protected double m_targetPosition = 0;

    public ElevatorMechanism(Limits limits, String name, PidConstants pid, FeedForwardConstants ff, double diffThreshold, int staleThreshold, double staleTolerance) {
        super(name, pid, ff, diffThreshold, staleThreshold, staleTolerance);
        m_limits = limits;

        toggleShowPIDTuning(false); //set to true to enable PID tuning on dashboard
    }

    @Override
    public void updatePeriodicSmartDashboard(){
        updateCounter++;
        if(updateCounter >= kUpdateInterval){
            updateCounter = 0;
            SmartDashboard.putNumber("Elevator Enc Pos", getElevatorPosition());
        }
        super.updatePeriodicSmartDashboard();
    }


    @Override
    public void periodic() {

        updatePeriodicSmartDashboard();

        if(!manualControl){
            RunElevator();
        }
    }


    public Boolean IsElevatorUp() {
        return getElevatorPosition() > m_limits.high-kLimitBuffer;
    }

    public Boolean IsElevatorDown() {
        return getElevatorPosition() < m_limits.low+kLimitBuffer;
    }

    @Override
    public void stopMechanism() {
        setSpeed(0);
    }

    public void stopElevator() {
        stopMechanism();
    }

    public Boolean isFinished(){
        return ReachedTarget(getElevatorPosition(), m_targetPosition);
    }

    private Boolean isFinishedEarly() {
        double delta = Math.abs(getElevatorPosition() - m_targetPosition);
        boolean isFinished =  ReachedTarget(getElevatorPosition(), m_targetPosition);
        return isFinished || delta < 3;
    }

    /**
     * Initialze the rotation of the elevator to a given position
     * @param position target position in rotations
     */
    public void InitMotion(double position){
        m_targetPosition = sanitizePositionSetpoint(position);
        manualControl = false;
        staleCounter = 0; //new target, reset stale counter
        SmartDashboard.putNumber("Elevator m_setpoint" + m_name, m_targetPosition);
    }    

    protected double sanitizePositionSetpoint(double setpoint){
        if (setpoint > m_limits.high){
            setpoint = m_limits.high;
        }
        if(setpoint < m_limits.low){
            setpoint = m_limits.low;
        }
        return setpoint;
    }

    /******************************************************
     * Abstract Methods to be written by the specific motor elevator class
     ******************************************************/
    public abstract void setSpeed(double speed);
    public abstract double getElevatorPosition();
    protected abstract double getRotationsPerSecond();
    protected abstract void resetEncoder();
    protected abstract void RunElevator();


  
    /*********************************************************
     * Commands
     *********************************************************/

    /**
     * @brief Runs the Elevator at a given speed (-1 to 1) in manual mode until interrupted
     * @param getSpeed a lambda that takes no arguments and returns the desired speed of the Elevator [ () => double ]
     * @return the composed command to manually drive the Elevator
     */
    public Command RunElevatorManualSpeedCommand(DoubleSupplier getSpeed) {
        return new FunctionalCommand(
                () -> {manualControl = true;},
                () -> setSpeed(getSpeed.getAsDouble()),
                (interrupted) -> stopElevator(),
                () -> false, this);
    }


    /**
     * @brief Runs the Elevator to a given position in absolute rotations
     * @param position absolute position to run to (not a lambda)
     * @return the composed command to run the Elevator to a given positions
     */
    public Command RunElevatorToPositionCommand(double position, Boolean async) {
        return (new FunctionalCommand(
                () -> {manualControl = false; InitMotion(position);},
                () -> {},
                (interrupted) -> {},
                () -> async || isFinished(), this)).withName("RunElevatorToPositionCommand");
    }

    public Command RunElevatorToPositionCommandEarlyFinish(double position) {
        return (new FunctionalCommand(
                () -> {manualControl = false; InitMotion(position);},
                () -> {},
                (interrupted) -> {},
                () -> isFinishedEarly(), this)).withName("RunElevatorToPositionCommand");
    }

    public Command ResetElevatorEncoderCommand() {
        return this.runOnce(() -> resetEncoder());
    }
    
}
