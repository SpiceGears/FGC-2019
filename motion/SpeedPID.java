package org.firstinspires.ftc.teamcode.motion;

import java.util.Timer;
//import edu.wpi.first.wpilibj.Timer;
import org.firstinspires.ftc.teamcode.Config;

public class SpeedPID {
    
    double kP;
    double kI;
    double kD; 
    double kF;
    double setSpeed;
    double error;
    double accualSpeed;
    
    double lastT;
    double errSum;
    double prevError;
    
    boolean reset;

    double maxOut = 0.95;
    double minOut = -0.95;
    double maxI = 0;
    double minI = 0;
    
    //Config config = new Config();
    
    
/*    public void SpeedPID() {
        kP = config.kP;
        kI = config.kI;
        kD = config.kD;
        kF = config.kF;
        
        if ( kI == 0 )
            maxI = 0;
        else
            maxI = 1/kI;
        minI = -maxI;
        
        reset();
    }*/
    
    public SpeedPID(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        
        if ( kI == 0 )
            maxI = 0;
        else
            maxI = 1/kI;
        minI = -maxI;
        
        reset();
    }
    
    public void setlastT() {
        lastT = System.currentTimeMillis() /1000;
    }

    public void setSetpoint(double setSpeed) {
        this.setSpeed = setSpeed;
    }
    
    public double getError() {
        if ( reset )
            return Double.MAX_VALUE;
        return error;
    }
    
    public double getInput() {
        return accualSpeed;
    }
    
    public void reset() {
        reset = true;
    }
    
    public double getSetpoint() {
        return setSpeed;
    }
    
    public void setOutputConstraints(double max, double min){
        maxOut = max;
        minOut = min;
    }
    
    public double calculate(double setSpeed, double accualSpeed) {
        //double now = Timer.getFPGATimestamp();
        double now = System.currentTimeMillis() /1000;
        double dt = now-lastT;
        lastT = now;
        return calculate(dt, setSpeed, accualSpeed);
    }
    
    public double calculate(double dT, double setSpeed, double accualSpeed) {
        //this.accualSpeed = accualSpeed;
        
        error = setSpeed - accualSpeed;
        
        errSum += error*dT;
        
        if ( errSum > maxI )
            errSum = maxI;
        else if ( errSum < minI )
            errSum = minI;

        double deltaPos = error-prevError;
        prevError = error;

        if ( reset ) {
            deltaPos = 0;
            lastT = System.currentTimeMillis();
            prevError = 0;
            errSum = 0;
            reset = false;
        }
        
        double out = (kP*error) + (kI*errSum) + (kD*(deltaPos/dT)) + (kF*setSpeed);

        
        // telemetry.addData("kP", kP*error);
        // telemetry.addData("kI",kI*errSum);
        // telemetry.addData("kD",kI*(deltaPos/dT));
        
        
        
        if ( out > maxOut )
            out = maxOut;
        else if ( out < minOut )
            out = minOut;
        
        return out;
    }
}
