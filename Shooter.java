package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.robot.Robot;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.motion.SpeedPID;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;

public class Shooter {

    
    boolean wasStartShoot = false;
    double oldFrontEncoder = 0;
    double oldBackEncoder = 0;
    double oldFrontTime = 0;
    double oldBackTime = 0;
    double frontRPM = 0;
    double backRPM = 0;
    double narutoRun = 0;
    
    double shootSpeed = 0;
    double oldTime = 0;
    String text = "nie wiem co robie";
    
    double maxSpeed = 0;
    double actualSpeed = 0;
    
    boolean isMaxSpeed = false;
    long startTimeShoot = 0;
    boolean isMaxVelocity = false;
    
    
    Config config;
    SpeedPID speedPID;
    
    public Shooter(Config config) {
        this.config = config;
        speedPID = new SpeedPID(config.shooterkP, config.shooterkI, config.shooterkD, config.shooterkF);
    }
    

    
    public void shoot2(Gamepad gamepad) {
        config.frontShoot.setPower(gamepad.right_stick_y * 0.25);
        config.backShoot.setPower(-gamepad.right_stick_y * 0.25);
        actualSpeed = config.backShoot.getVelocity();
    }
    
    public void shootLinear(Gamepad gamepad) {
        if(gamepad.dpad_down) {
            if(!isMaxVelocity) {
                if(config.frontShoot.getVelocity() < 400) {
                    text = "troche wiem co robie";
                    config.frontShoot.setPower(0.1);
                    config.backShoot.setPower(0.1);
                    
                }else if(config.frontShoot.getVelocity() < 1200){
                    text = "little more i understand";
                    config.frontShoot.setPower(0.25);
                    config.backShoot.setPower(0.25);
                    
                }else if(config.frontShoot.getVelocity() < 1800) {
                    text = "jezcze wiecej wiem co robie";
                    config.frontShoot.setPower(0.4);
                    config.backShoot.setPower(0.4);
                }else if(config.frontShoot.getVelocity() < 2350) {
                    config.frontShoot.setPower(0.6);
                    config.backShoot.setPower(0.6);
                }else {
                    isMaxVelocity = true;
                }
            }else {
                VuforiaBase.TrackingResults vuforiaResults = config.vuforia.track(config.TARGET_TO_TRACK);
                if(vuforiaResults.y < -920) {
                    double deltaDistance = vuforiaResults.y + 1000;
                    narutoRun = deltaDistance *0.66;
                    narutoRun = 0;
                }
                //text = "kompletnie wiem co robie";
                config.frontShoot.setVelocity(2350 - narutoRun);
                config.backShoot.setVelocity(2350 - narutoRun);
            }
        }else {
            
            config.frontShoot.setVelocity(0);
            config.backShoot.setVelocity(0);
            oldTime = System.currentTimeMillis();
            isMaxVelocity = false;
        }
            
    }
    
    
    public void shootPID(Gamepad gamepad) {
        if(gamepad.x) {
            config.backShoot.setVelocity(-2400);
            config.frontShoot.setVelocity(-2400);
        }else {
            config.frontShoot.setVelocity(0);
            config.backShoot.setVelocity(0);
        }
        actualSpeed = config.backShoot.getVelocity();
    }
    
    public void shootPIDV(int velocity) {
        config.backShoot.setVelocity(-velocity);
        config.frontShoot.setVelocity(-velocity);
    }
    
    public void shootStop() {
        config.backShoot.setVelocity(0);
        config.frontShoot.setVelocity(0);
    }
    
    public void shootXX() {
        if(wasStartShoot){
            startTimeShoot = System.currentTimeMillis();
        } else {
            double now = System.currentTimeMillis();
            double delta = now - startTimeShoot;
            
            double maxPower = -0.1;
            double speed = delta / 2000;
            
            if(Math.abs(delta) >= 2000){
                isMaxSpeed = true;
            }
                config.backShoot.setPower(speed * maxPower);
                config.frontShoot.setPower(speed * maxPower);
                
            
        }
    }
    
    public void shootPID22(Gamepad gamepad) {
        if(gamepad.dpad_up) {
            if(isMaxSpeed){
                config.backShoot.setVelocity(-2200);
                config.frontShoot.setVelocity(-2200);
            }
            else
            {
               shootXX(); 
               //shoot(gamepad); 
            }
        }else {
            config.frontShoot.setVelocity(0);
            config.backShoot.setVelocity(0);
            isMaxSpeed = false;
            wasStartShoot = false;

        }
        //actualSpeed = config.backShoot.getVelocity();
    }
    
    public double calculateFrontRPM() {
        double newEncoder = config.frontShoot.getCurrentPosition();
        double newTime = System.currentTimeMillis();
        
        if(newTime - oldFrontTime > 100) {
            oldFrontTime = newTime;
            double deltaRotation = newEncoder - oldFrontEncoder;
            oldFrontEncoder = newEncoder;
            frontRPM = deltaRotation / 244 * 600;
        }
        return frontRPM;
        
    }
   
   
    public double calculateBackRPM() {
        double newEncoder = config.backShoot.getCurrentPosition();
        double newTime = System.currentTimeMillis();
        
        if(newTime - oldBackTime > 100) {
            oldBackTime = newTime;
            double deltaRotation = newEncoder - oldBackEncoder;
            oldBackEncoder = newEncoder;
            backRPM = deltaRotation / 244 * 600;
        }
        return backRPM;
        
    }

    // todo: write your code here
}









/*
      #
    #####
  #########
 ###########
      =
      
      */
