package frc.robot.mechanisms;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Types.FeedForwardConstants;
import frc.robot.utils.Types.PidConstants;

public abstract class Mechanism extends SubsystemBase{
    private double kDiffThreshold;
    private double kStaleThreshold;
    private double kStaleTolerance;
    protected int staleCounter;
    private double lastPosition;
    protected boolean manualControl;

    // PID controller and values for testing
    private boolean showPIDTuning = false;
    protected PidConstants usedPID = new PidConstants(0,0,0);
    protected double usedG;
    protected double usedV;
    protected final String m_name;

    /**
     * 
     * @param diffThreshold - the difference threshold between desired and current positions
     *                        to consider the mechanism at its target
     * @param staleThreshold - the number of cycles the mechanism must be within the
     *                        staleTolerance to consider it at its target
     * @param staleTolerance - the tolerance within which the mechanism must be away from its
     *                         last position to consider it stale and stalled
     * 
     * @note in general, diffThreshold should be less than staleTolerance
     */
    public Mechanism(String name, PidConstants pid, FeedForwardConstants ff, double diffThreshold, double staleThreshold, double staleTolerance){
        kDiffThreshold = diffThreshold;
        kStaleThreshold = staleThreshold;
        kStaleTolerance = staleTolerance;
        staleCounter = 0;
        manualControl = true; //start with the mechanism in manual control to prevent sudden jolts
        lastPosition = 0;
        m_name = name;

        //set variables for PID tuning on smart dashboard
        usedPID = pid;
        usedG = ff.kg;
        usedV = ff.kv;
    }

    public void toggleShowPIDTuning(Boolean enable){
        showPIDTuning = enable;
    }

    public void updatePeriodicSmartDashboard(){
       if(showPIDTuning){
            double p = SmartDashboard.getNumber(m_name + " P Gain", 0);
            double i = SmartDashboard.getNumber(m_name + " I Gain", 0);
            double d = SmartDashboard.getNumber(m_name + " D Gain", 0);
            double g = SmartDashboard.getNumber(m_name + " Gravity FF", 0);
            double v = SmartDashboard.getNumber(m_name + " Velocity FF", 0);

            boolean valueAdjusted = false;
        
            if((p != usedPID.p)) { valueAdjusted = true; usedPID.p = p; }
            if((i != usedPID.i)) { valueAdjusted = true; usedPID.i = i; }
            if((d != usedPID.d)) { valueAdjusted = true; usedPID.d = d; }
            if((g != usedG)) { valueAdjusted = true; usedG = g; }
            if((v != usedV)) { valueAdjusted = true; usedV = v; }
            if(valueAdjusted){
                updatePIDGV(usedPID, g, v);
            }
       }
        
    }

    public Boolean ReachedTarget(double currentPosition, double finalPosition){

        // check if the mechanism has stalled and is no longer moving
        // if it hasn't moved (defined by encoder change less than kDiffThreshold),
        // increment the stale counter
        // if it has moved, reset the stale counter
        if (Math.abs(currentPosition - lastPosition) < kStaleTolerance) {
            staleCounter++;
        } else {
            staleCounter = 0;
        }
        lastPosition = currentPosition;

        // calculate the difference between the current position and the final position
        double delta = Math.abs(currentPosition - finalPosition);

        // we say that the mechanism has reached its target if it is within
        // kDiffThreshold of the target,
        // or if it has been within a looser kStaleTolerance for kStaleThreshold cycles
        return delta < kDiffThreshold
                || (delta < kStaleTolerance && staleCounter > kStaleThreshold);
    }

   

    public void resetAfterDisable(){
        System.out.println("Reset Mech After Disable!!!");
        stopMechanism();
        manualControl = true;
    }
    
    public abstract void stopMechanism();

    public abstract void updatePIDGV(PidConstants pid, double g, double v);
    

}
