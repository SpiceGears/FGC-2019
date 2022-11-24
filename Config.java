/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaBase;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaFG2019;


public class Config
{
    /* Public OpMode members. */
    public DcMotorEx  leftDrive   = null;
    public DcMotorEx  rightDrive  = null;

    public DcMotorEx  leftElevator  = null;
    public DcMotorEx  rightElevator = null;

    public DcMotor  leftIntake  = null;
    public DcMotor  rightIntake = null;

    public DcMotorEx  frontShoot = null;
    public DcMotorEx  backShoot  =null;
    
    public CRServo leftFeeder = null;
    public CRServo rightFeeder = null;
    
    public ColorSensor leftColor = null;
    public ColorSensor rightColor = null;
    
    public CRServo poleczka = null;
    
    public Servo wspinanie = null;
    
    public TouchSensor touch = null;
    
    public AnalogInput potentiometer = null;
    
    public VuforiaFG2019 vuforia;
    public WebcamName webcam;
    private final boolean USE_EXTENDED_TRACKING = false;
    private final boolean ENABLE_CAMERA_DISPLAY = false;
    private final String DUBAI_2019_TARGET = VuforiaFG2019.DUBAI_2019_TARGET;
    public final String WORLD_MAP_TARGET = VuforiaFG2019.WORLD_MAP_TARGET;
    public final String TARGET_TO_TRACK = WORLD_MAP_TARGET;
    
    private final float CAMERA_LOCATION_ON_ROBOT_X_MM = 180;
    private final float CAMERA_LOCATION_ON_ROBOT_Y_MM = 0;
    private final float CAMERA_LOCATION_ON_ROBOT_Z_MM = 0;
    private final float CAMERA_ANGLE_X_DEGREES = -90;
    private final float CAMERA_ANGLE_Y_DEGREES = 0;
    private final float CAMERA_ANGLE_Z_DEGREES = 0;
    
    
    public double elevatorkP = 20;
    public double elevatorkI = 1.5;
    public double elevatorkD = 0.5;
    public double elevatorkF = 0.1;
    public double elevatordT = 0;
    
    public double shooterkP = 0;
    public double shooterkI = 0;
    public double shooterkD = 0;
    public double shooterkF = 0;
    public double shooterdT = 0;
    
    
    //public double position1 = 0;
    //public double position2 = 900;

//    public static final double MID_SERVO       =  0.5 ;
//    public static final double ARM_UP_POWER    =  0.45 ;
//    public static final double ARM_DOWN_POWER  = -0.45 ;

    /* local OpMode members. */
    HardwareMap hwMap           =  null;
    private ElapsedTime period  = new ElapsedTime();

    /* Constructor */
    public Config(){

    }

    /* Initialize standard Hardware interfaces */
    public void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;
        
        vuforia = new VuforiaFG2019();
        webcam = hwMap.get(WebcamName.class, "Webcam 1");
        
        vuforia.initialize(
                webcam,                         // Our webcam that we retrieved from the HardwareMap
                USE_EXTENDED_TRACKING,          // Whether or not Vuforia should attempt to track the target when it cannot see it
                ENABLE_CAMERA_DISPLAY,          // Whether or not Vuforia should display the camera output on a connected HDMI monitor
                CAMERA_LOCATION_ON_ROBOT_X_MM,  // The X location of the camera on the robot's coordinate system, in millimeters
                CAMERA_LOCATION_ON_ROBOT_Y_MM,  // The Y location of the camera on the robot's coordinate system, in millimeters
                CAMERA_LOCATION_ON_ROBOT_Z_MM,  // The Z location of the camera on the robot's coordinate system, in millimeters
                CAMERA_ANGLE_X_DEGREES,         // The amount to rotate the camera along the robot's X axis
                CAMERA_ANGLE_Y_DEGREES,         // The amount to rotate the camera along the robot's Y axis
                CAMERA_ANGLE_Z_DEGREES);
        vuforia.activate();
        
        VuforiaBase.TrackingResults vuforiaResults;
        
        //touch = hwMap.get(TouchSensor.class, "elevatorLimit");

        // Define and Initialize Motors
        leftDrive  = hwMap.get(DcMotorEx.class, "leftDrive");
        rightDrive = hwMap.get(DcMotorEx.class, "rightDrive");

        leftElevator = hwMap.get(DcMotorEx.class, "leftElevator");
        rightElevator = hwMap.get(DcMotorEx.class, "rightElevator");

        leftIntake = hwMap.get(DcMotor.class,"leftIntake");
        rightIntake = hwMap.get(DcMotor.class,"rightIntake");

        frontShoot = hwMap.get(DcMotorEx.class, "frontShoot");
        backShoot = hwMap.get(DcMotorEx.class, "backShoot");
        
        leftFeeder = hwMap.get(CRServo.class, "leftFeeder");
        rightFeeder = hwMap.get(CRServo.class, "rightFeeder");
        
        poleczka = hwMap.get(CRServo.class, "poleczka");
        
        potentiometer = hwMap.get(AnalogInput.class, "potentiometer");
//244
        rightColor = hwMap.get(ColorSensor.class, "rightColor");
        leftColor = hwMap.get(ColorSensor.class, "leftColor");

        wspinanie = hwMap.get(Servo.class, "wspinanie");

        leftDrive.setDirection(DcMotor.Direction.REVERSE); // Set to REVERSE if using AndyMark motors
        rightDrive.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors

        rightElevator.setDirection(DcMotor.Direction.REVERSE);

        leftIntake.setDirection(DcMotor.Direction.REVERSE);
        
        backShoot.setDirection(DcMotor.Direction.REVERSE);
        frontShoot.setDirection(DcMotor.Direction.REVERSE);

        // Set all motors to zero power
        leftDrive.setPower(0);
        rightDrive.setPower(0);

        leftElevator.setPower(0);
        rightElevator.setPower(0);
        
        leftIntake.setPower(0);
        rightIntake.setPower(0);
        
        frontShoot.setPower(0);
        backShoot.setPower(0);
        
        
        
        
        



        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        leftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //leftElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightElevator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //leftElevator.setTargetPosition(0);
        //rightElevator.setTargetPosition(0);
        
        leftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        rightElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        //leftElevator.setVelocity(300);
        //rightElevator.setVelocity(300);
        
        leftElevator.setVelocityPIDFCoefficients(8.3, 0.63, 0, 63.0);
        rightElevator.setVelocityPIDFCoefficients(8.3, 0.63, 0, 63.0);

        //RUN_TO_POSITION
        //RUN_USING_ENCODERS
        
        frontShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backShoot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        
        frontShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        backShoot.setMode(DcMotor.RunMode.RUN_USING_ENCODERS);
        
        backShoot.setVelocityPIDFCoefficients(12, 4, 0.5, 0);
        frontShoot.setVelocityPIDFCoefficients(12, 4, 0.5, 0);

//        leftClaw  = hwMap.get(Servo.class, "left_hand");
//        rightClaw = hwMap.get(Servo.class, "right_hand");
//        leftClaw.setPosition(MID_SERVO);
//        rightClaw.setPosition(MID_SERVO);
    }
 }

