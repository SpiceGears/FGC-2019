package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.motion.SpeedPID;

public class Elevator {
    Config config;
    
    long encoderPosition;
    
    double maxSpeed = 0;
    double actualSpeed = 0;
    int actualPosition = 0;
    
    SpeedPID speedPID;
    
    //double newSetpoint = config.position1;
    
    public Elevator(Config config) {
        this.config = config;
        speedPID = new SpeedPID(config.elevatorkP,config.elevatorkI,config.elevatorkD,config.elevatorkF);
        speedPID.setlastT();
    }
    
    
    
    public void controllWithJoystick(Gamepad gamepad) {
        encoderPosition = -config.rightElevator.getCurrentPosition();
        double yValue = gamepad.left_stick_y;
        if(Math.abs(yValue) > 0.1){
            config.leftElevator.setPower(yValue);
            config.rightElevator.setPower(yValue);
        }else {
            config.leftElevator.setPower(0);
            config.rightElevator.setPower(0);
        }
        
        actualSpeed = -config.rightElevator.getVelocity();
        
        if(actualSpeed > maxSpeed) {
            maxSpeed = actualSpeed;
        }
    }
    
    public void controllWithButton(Gamepad gamepad) {
        encoderPosition = -config.rightElevator.getCurrentPosition();
        if(gamepad.a) {
            config.leftElevator.setTargetPosition(-750);
            config.rightElevator.setTargetPosition(-750);
/*            config.leftElevator.setVelocity(300);
            config.rightElevator.setVelocity(300);*/
        }else if(gamepad.b) {
            config.leftElevator.setTargetPosition(-10);
            config.rightElevator.setTargetPosition(-10);
        }
    }
    
    public void controllWithStick(Gamepad gamepad) {
        if(Math.abs(gamepad.left_stick_y) > 0.1) {
            actualPosition -= gamepad.left_stick_y * 10;
             if(actualPosition < 0) {
                actualPosition = 0;
            }
            
            if(actualPosition > 750) {
                actualPosition = 750;
            } 
            
            /*if(config.touch.isPressed()) {
                config.leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                config.ri`g`htElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                config.leftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                config.rightElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
            }*/
            config.leftElevator.setTargetPosition(-actualPosition);
            config.rightElevator.setTargetPosition(-actualPosition);
        }
    }
    
}
