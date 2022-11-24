package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;

@TeleOp

public class Robot2 {
    private AnalogInput potentiometer;
    private Gyroscope imu;
    private CRServo rightFeeder;
    private CRServo poleczka;
    private CRServo leftFeeder;
    private DcMotor frontShoot;
    private DcMotor leftIntake;
    private DcMotor leftElevator;
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private DcMotor rightElevator;
    private DcMotor rightIntake;
    private DcMotor backShoot;
    private DistanceSensor distance;
    private Blinker control_Hub;
    private Blinker expansion_Hub_;
    private DistanceSensor rightColor;
    private DistanceSensor leftColor;
    private Servo wspinanie;
    private TouchSensor rightTouch;
    private HardwareDevice webcam_1;

    // todo: write your code here
}


@TeleOp(name="2021 Test", group="Pushbot")
public class Robot extends OpMode{
    
    /*
    DO ZROBIENIA
        -Zliczanie ilość piłek w jednym i drugim magazynku na małe piłki USELESS
        -Limit aby więcej nie zebrał       USELESS
        -Konfiguracja ile ma się obrócić serwo.  DONE
        -Specjalny reset do pozycji startowej.   DONE
        -Kamera, ustawianie kąta. Done
        -Setpointy na windę 2 i 3 poziom. ALMOST DONE
        -Podciąganie 
        -Servo mechanizm do wysypywania dużych piłek DONE
    */
    
    
     /*
        Drive: gamepad1: left and right stick
        brake: gamepad1: y
        drive to target: gamepad1: b
        intake: gamepad1: left and right bumpers
        shooter: gamepad2: x
        poleczka: gamepad2: b
        elevator: gamepad2: left stick
        left_dpad: wspinanie
        left trigger: wspinanie winda
        
    
    */

    /* Declare OpMode members. */
    long oldTime = 0;
    private double speed = 0;
    int sum = 0;
    boolean wasStartShoot = false;
    long startTimeShoot = 0;
    long actualTime;
    long endTime = 0; 
    long feederLeftNewTime = 0;
    long feederRightNewTime = 0;
    boolean changer = true;
    
    double poleczkaPosition =  1.30;
    
    Config config = new Config();
    
    Shooter shooter = new Shooter(config);
    
    Elevator elevator = new Elevator(config);
    
    //SpeedPID speedPID = new SpeedPID();
    
    // LinkedList<Double> xLeft = new LinkedList<Double>();
    // LinkedList<Double> yLeft = new LinkedList<Double>();
    // LinkedList<Double> xRight = new LinkedList<Double>();
    // LinkedList<Double> yRight = new LinkedList<Double>();

    @Override
    public void init() {
        config.init(hardwareMap);
        // for(int i = 0; i < 70; i++) {
        //     xLeft.add(0.0);
        //     yLeft.add(0.0);
        //     xRight.add(0.0);
        //     yRight.add(0.0);
        // }
    }

  
  
    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        
        oldTime = System.currentTimeMillis();
    }


    @Override
    public void loop() {
        //shooter.shootPID22(gamepad2);
        shooter.shootLinear(gamepad2);
        intake();
        if(gamepad2.left_trigger) {elevator.controllWithJoystick(gamepad2);} else { config.leftElevator.setPower(0);
                                                                                    config.rightElevator.setPower(0);}/// else {elevator.controllWithButton(gamepad2); }
        //if(gamepad1.b) driveToTarget();
        //else brake();
        brake();
        //drive();
        if(gamepad2.left_bumper || gamepad2.right_bumper || gamepad2.dpad_up ||gamepad2.right_trigger) shootFeeder();
        else feeder();
        //elevator.controllWithButton(gamepad2);//controllWithStick
        cameraDetection();
        poleczka();
        climbing();
        //colorDetection();
        telemetry();
    }

    
    
    public void cameraDetection() {
        VuforiaBase.TrackingResults vuforiaResults = config.vuforia.track(config.TARGET_TO_TRACK);
        double distance = Math.sqrt(Math.pow(vuforiaResults.x,2) + Math.pow(vuforiaResults.y,2));
        double actualAngle = vuforiaResults.zAngle;
        double targetAngle = Math.atan(vuforiaResults.x / vuforiaResults.y) * 180 / Math.PI;
        
        telemetry.addData("Is visible", vuforiaResults.isVisible);
        telemetry.addData("coordinate:", "x = " + vuforiaResults.x + " y = " + vuforiaResults.y);
        telemetry.addData("angle:", "xAngle = " + vuforiaResults.xAngle + "yAngle = " + vuforiaResults.yAngle + "zAngle = " + vuforiaResults.zAngle);
        telemetry.addData("distance ",distance);
        telemetry.addData("targetAngle",targetAngle);
        
    }


    public void intake() {
        if(gamepad1.right_bumper == true && gamepad1.left_bumper == false) {
            config.rightIntake.setPower(1.0);
            config.leftIntake.setPower(1.0);
        } else if(gamepad1.right_bumper == false && gamepad1.left_bumper == true) {
            config.rightIntake.setPower(-1.0);
            config.leftIntake.setPower(-1.0);
        }else {
            config.rightIntake.setPower(0);
            config.leftIntake.setPower(0);
        }
    }
    
    public void shootFeeder() {
        if(gamepad2.right_bumper) {
            config.leftFeeder.setPower(-1);
        }else {
            config.leftFeeder.setPower(0);
        }
        if(gamepad2.left_bumper) {
            config.rightFeeder.setPower(1);
        }else {
            config.rightFeeder.setPower(0);
        }
        if(gamepad2.dpad_up) {
            config.rightFeeder.setPower(-1);
            config.leftFeeder.setPower(1);
        }
        
    }
    
    
    public void climbing() {
        if(gamepad2.dpad_right) {
            config.wspinanie.setPosition(0.21);
        }else if(gamepad2.x) {
            config.wspinanie.setPosition(0.7);
        }
    }
    
    
    
    public void feeder() {
        if(config.leftColor.alpha() > 200) {
            feederLeftNewTime = System.currentTimeMillis() + 500;
        }
        
        if(config.rightColor.alpha() > 200) {
            feederRightNewTime = System.currentTimeMillis() + 500;
        }
        
        if(feederLeftNewTime > System.currentTimeMillis()) {
            config.leftFeeder.setPower(-1);
        }else {
            config.leftFeeder.setPower(0);
        }
        
        if(feederRightNewTime > System.currentTimeMillis()) {
            config.rightFeeder.setPower(1);
        }else {
            config.rightFeeder.setPower(0);
        }
    }
    
    
    public void poleczka() {
        //jak łuki podepnie to się zrobi
        double narutoRun = 1.2;
        if(gamepad2.b) {
            poleczkaPosition = 2.2;
            narutoRun = 1.2;
        } else if(gamepad2.a) {
            poleczkaPosition = 3.5;
            narutoRun = 9;
        }else if(gamepad2.y) {
            poleczkaPosition = 1.50;
            narutoRun = 1.2;
        }
        double error = poleczkaPosition - config.potentiometer.getVoltage();
        double power = error * narutoRun;
        
        config.poleczka.setPower(-power);
        
    }
    
    public void colorDetection() {
        telemetry.addData("Red",config.leftColor.red());
        telemetry.addData("Green",config.leftColor.green());
        telemetry.addData("Blue",config.leftColor.blue());
        telemetry.addData("Alpha",config.leftColor.alpha());
        
        telemetry.addData("Red",config.rightColor.red());
        telemetry.addData("Green",config.rightColor.green());
        telemetry.addData("Blue",config.rightColor.blue());
        telemetry.addData("Alpha",config.rightColor.alpha());
        
    }
    
    public void telemetry() {
        
        //telemetry.addData("list Lenght", xLeft.size());
        //telemetry.addData("shooter encoder", config.frontShoot.getCurrentPosition());
        //telemetry.addData("shooter Front Velocity", shooter.actualSpeed);
        //telemetry.addData("shooter speed1", shooter.actualSpeed);
        //telemetry.addData("frontShoot", config.frontShoot.getCurrentPosition());
        //telemetry.addData("backShoot",config.backShoot.getCurrentPosition());
        //telemetry.addData("shooter Max speed", shooter.maxSpeed);
        telemetry.addData("potentiometer position", config.potentiometer.getVoltage());
        //telemetry.addData("servo Power", config.poleczka.getPower());
        telemetry.addData("elevator velocity", config.leftElevator.getVelocity());
        telemetry.addData("nie wiem co robie", config.rightDrive.getCurrentPosition());


        telemetry.addData("front  shoot velocity", config.frontShoot.getVelocity());
        telemetry.addData("back  shoot velocity", config.backShoot.getVelocity());
        telemetry.addData("narutoRun",shooter.narutoRun);
    }
    
     public void brake() {
        if(gamepad1.right_trigger) {
            if(changer) {
                config.leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                config.rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                config.leftDrive.setTargetPosition(0);
                config.rightDrive.setTargetPosition(0);
                
                config.leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                config.rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                
                config.leftDrive.setVelocityPIDFCoefficients(6.3, 0.63, 0, 63.01);
                config.rightDrive.setVelocityPIDFCoefficients(6.3, 0.63, 0, 63.01);
                telemetry.addData("jebać disa", 0);
            }
            config.leftDrive.setVelocity(300);
            config.rightDrive.setVelocity(300);
            telemetry.addData("drive target position",config.leftDrive.getTargetPosition());
            telemetry.addData("drive current position",config.leftDrive.getCurrentPosition());
            changer = false;
        }else {
            if(!changer) {
                config.leftDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                config.rightDrive.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                config.leftDrive.setPower(0);
                config.rightDrive.setPower(0);
            }
            drive();
            changer = true;
        }
        telemetry.addData("changer", changer);
    }
    
    public void drive() {
        double maxSpeed = 1;
        sum = 0;
        long newTime = System.currentTimeMillis();
        double deltaTime = newTime - oldTime;
        double joySpeedValue = -gamepad1.left_stick_y * maxSpeed ; //0.7
        
        if(gamepad1.right_bumper){
            joySpeedValue *= 0.6;//0.8
        }
        
        //SmartDashboard.putNumber("encoderRight", getDistanceInMeters());
        // double leftY = gamepad1.left_stick_y;
        // double leftX = gamepad1.left_stick_x;
        // double rightY = gamepad1.right_stick_y;
        // double rightX =gamepad1.right_stick_x;
        
        
        //This value reach 100% power motor in 0,5s
        double speedAddicional = 0.10; // 0.07
        speedAddicional = (deltaTime/50) * speedAddicional;

        if(Math.abs(joySpeedValue) < 0.2){
            joySpeedValue = 0;
        }
        
        if(-joySpeedValue > speed && speed < 0) {
            speed = speed + speedAddicional * 2.5;
            oldTime = System.currentTimeMillis();
        } else if(-joySpeedValue >= speed && speed >= 0){
            speed = speed + speedAddicional;
            oldTime = System.currentTimeMillis();
        }else if(-joySpeedValue < speed && speed > 0) {
            speed = speed - speedAddicional * 2.5;
            oldTime = System.currentTimeMillis();
        }else if (-joySpeedValue <= speed && speed <= 0){
            speed = speed - speedAddicional;
            oldTime = System.currentTimeMillis();
        }else {
            speed = speed;
            oldTime = System.currentTimeMillis();
        }

        if (speed > maxSpeed){
            speed = maxSpeed;
        }
        else if (speed < -maxSpeed){
            speed = -maxSpeed;
        }

        double turn = -gamepad1.right_stick_x * 0.5; //0.3 0.4
        
        
        if(Math.abs(turn) < 0.1){
            turn = 0;
        }
        
        if(gamepad1.right_bumper) {
            turn *= 0.8;
        }

        if(speed < 0.2) {
            turn *= 1.2;
        }
        double newspeed = 0;
        if(gamepad1.left_trigger) {
            newspeed = speed * 1.9;    
        }else {
            newspeed = speed;
        }
        
        double left = newspeed + turn;
        double right = newspeed - turn;
        
        if(Math.abs(left) < 0.10) {
            left = 0;
        }
        
        if(Math.abs(right) < 0.10) {
            right = 0;
        }
        
        // for(int i = 0; i <70;i++) {
        //     if(gamepad1.left_stick_x == xLeft.get(i)) sum++;
        //     if(gamepad1.left_stick_y == yLeft.get(i)) sum++;
        //     if(gamepad1.right_stick_x == xRight.get(i)) sum++;
        //     if(gamepad1.right_stick_y == yRight.get(i)) sum++;
        // }
        // if(sum == 280) {
        //     left = 0;
        //     right = 0;
        // }
        // xLeft.removeFirst();
        // yLeft.removeFirst();
        // xRight.removeFirst();
        // yRight.removeFirst();
        
        // xLeft.add(leftX);
        // yLeft.add(leftY);
        // xRight.add(rightX);
        // yRight.add(rightY);
        
        if((gamepad1.right_stick_x <0.1 && gamepad1.right_stick_x > -0.1) && (gamepad1.left_stick_y < 0.2 && gamepad1.left_stick_y > -0.2)){
            config.leftDrive.setPower(0);
            config.rightDrive.setPower(0);
        }else{
            config.leftDrive.setPower(left);
            config.rightDrive.setPower(right);
        }
        
        telemetry.addData("sum = ", sum);
        telemetry.addData("Delta Time", deltaTime);
        telemetry.addData("newSpeed", newspeed);
        telemetry.addData("speed",speed);
        telemetry.addData("turn", turn);
        telemetry.addData("left Drive speed", left);
        telemetry.addData("right Drive speed", right);
        telemetry.addData("left Drive power", config.leftDrive.getPower());
        telemetry.addData("right Drive power", config.rightDrive.getPower());
        telemetry.addData("left Drive encoder", config.leftDrive.getCurrentPosition());

    }
    
}
