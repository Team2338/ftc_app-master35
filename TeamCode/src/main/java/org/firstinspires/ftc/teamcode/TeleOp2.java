/**
 * Created by Mach Speed Programming on 10/31/2017.
 */
//
//TELEOP WITH CODE FOR COLOR AND DISTANCE AS WELL AS TOUCH SENSOR
//
// 03-MAR-2018 - Added Joystick Button to Wiggle Robot
//                 Added DPAD Left and Right to test Glyph arm rotate
//                         Added DPAD Up to test Encoder move onto board
// 11-FEB-2018 - Added servo4 and servo5 for new upper glyph grabbers
// 09-FEB-2018 - Commented out 2 front distance sensors
// 02-FEB-2018 - Added Full Glyph Open to RIght Bumper
//
// 17-JAN-2018 - Combined multiple GamePad1 Y IF's into 1 IF Statement
//		Added Multiplier Parameters for faster speed changes
//
// 11-JAN-2018 - Corrected servo Open and Close values
//
//
package org.firstinspires.ftc.teamcode;
//import com.ftdi.j2xx.D2xxManager;
//import com.qualcomm.robotcore.robocol.PeerApp;

import android.app.Activity;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;  //Needed for Color Sensor


@TeleOp(name="TeleOp2", group="Iterative OpMode")  // @Autonomous(...) is the other common choice
//@Disabled

public class TeleOp2 extends OpMode {
    /* Declare OpMode members. */

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontleftmotor;
    DcMotor frontrightmotor;
    DcMotor backleftmotor;
    DcMotor backrightmotor;

    DcMotor liftmotor;
    DcMotor relicUp;
    DcMotor relicOut;

    DcMotor rotateglyph;

    Servo servo0;	//Bottom Right Grabber
    Servo servo1;	//Bottom Left Grabber
    Servo servo2;	//Color Arm
    Servo servo3;	//Relic Claw
    Servo servo4;	//Top Right Grabber
    Servo servo5;	//Top Left Grabber
    Servo servo6;   //Color Arm pt2


    DigitalChannel digitaltouch;  // Hardware Device Object

    ColorSensor sensorcolor;
    DistanceSensor sensordistance;

    //ColorSensor sensorcolor2;		//USE FOR ROTATE ARM??
    //DistanceSensor sensordistance2;

    //ColorSensor sensorcolor3;
    //DistanceSensor sensordistance3;


    String ENCODERROTATE0 	= "YES";  //USED TO SWITCH GLYPH GRABBERS FROM TOP TO BOTTOM


    //
    // ROTATE GLYPH, ENCODER VALUES
    //
    int rotateglyphto0 	    = 0;
    int rotateglyphto180    = -850; //was -850

    double rotateglyphspeed = .5;

    double speedreduction;
    double speedfast		= 1.1; //was 1.5
    double speedslow		= 4;
    double speednormal		= 1.5;	//was 1.5 trying to slow down a little

    double frontleftspeed;
    double backleftspeed;

    double frontrightspeed;
    double backrightspeed;
    //
    //
    // SERVO CODE
    //
    // LOWER COLOR ARM
    // .75 is about perfect position lowering color arm
    // .55 is probably too high off the ground
    //
    static final double ColorDown     =  .75;     // Maximum rotational position LOWER .75

    // RAISE COLOR ARM
    // .2 is about perfect position for raising color arm
    // .1 raises it too much
    // .3 arm ends up way too low in the raised position
    //
    static final double ColorUp     =  0;     // Minimum rotational position RAISE 0.2


    //BOTTOM RIGHT GRABBER ARM
    static final double Servo0Open 	    = .65;    	//Bottom Right Open   	1
    static final double Servo0Close 	= .45;     	//Bottom Right Close  	0

    //BOTTOM LEFT GRABBER ARM
    static final double Servo1Close 	= .55;    	//Top Left Close       	1
    static final double Servo1Open 	    = .35;     	//Top Left Open        	0

    static final double RelicOpen      = .5;        //RELIC OPEN
    static final double RelicClose     = .8;        //RELIC CLOSE
    //BOTTOM SUPER CLOSE
    static final double Servo0FullClose = .3;  		//Right Super close
    static final double Servo1FullClose = .85;  	//Left Super Close


    //MOTOR MULTIPLIERS
    double liftmotormultiplier		= 0.8;      //Glyph Lift Motor Multiplier
    double relicUpmultiplier		= 0.5;      //Relic Arm Lift Motor Multiplier

    double slowbuttonmultiplier		= 0.3;      //Drive Motor Slow Button Multiplier was .25
    double normalbuttonmultiplier	= 0.7;      //Drive Motor Slow Button Multiplier was .5
    double fastbuttonmultiplier		= 0.2;      //Drive Motor Slow Button Multiplier

    int getcurposfrontright;			//Save encoder value for encoder braking
    int getcurposfrontleft;
    int getcurposbackright;
    int getcurposbackleft;

    int count1;				//Count for Encoder Braking
    int buttoncount 	= 0;		//Test of a multibutton
    int loop_count 	    = 0;		//Count for Debugging Loop
    int dpadcount1 	    = 0;		//Count for Debugging Loop
    int chuckcount      = 0;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        telemetry.addData("Status: ", "Initialized");
        telemetry.update();

        /* eg: Initialize the hardware variables. Note that the strings used here as parameters
         * to 'get' must correspond to the names assigned during the robot configuration
         * step (using the FTC Robot Controller app on the phone).
         */
        frontleftmotor = hardwareMap.dcMotor.get("frontleftdrive");
        frontrightmotor = hardwareMap.dcMotor.get("frontrightdrive");
        backleftmotor = hardwareMap.dcMotor.get("backleftdrive");
        backrightmotor = hardwareMap.dcMotor.get("backrightdrive");

        liftmotor = hardwareMap.dcMotor.get("liftmotor");
        relicOut = hardwareMap.dcMotor.get("relicOut");
        relicUp  = hardwareMap.dcMotor.get("relicUp");

        rotateglyph  = hardwareMap.dcMotor.get("rotateglyph");

        servo0 = hardwareMap.servo.get("servo0"); //Top Right Grabber
        servo1 = hardwareMap.servo.get("servo1"); //Top Left Grabber
        servo2 = hardwareMap.servo.get("servo2"); //Color Arm
        servo3 = hardwareMap.servo.get("servo3"); //Relic Claw
        servo4 = hardwareMap.servo.get("servo4"); //Bottom Right Grabber
        servo5 = hardwareMap.servo.get("servo5"); //Bottom Left Grabber
        servo6 = hardwareMap.servo.get("servo6"); //Bottom Left Grabber

        count1 		= 1;		//Count for Encoder Braking
        loop_count 	= 0;		//Testing how the loop works
        dpadcount1 	= 1;		//Count for Encoder Move On Board
        chuckcount  = 0;

        // get a reference to the color sensor.
        //sensorcolor2 = hardwareMap.get(ColorSensor.class, "sensorcolordistanceright");
        // get a reference to the distance sensor that shares the same name.
        //sensordistance2 = hardwareMap.get(DistanceSensor.class, "sensorcolordistanceright");

        // get a reference to the color sensor.
        //sensorcolor3 = hardwareMap.get(ColorSensor.class, "sensorcolordistanceleft");
        // get a reference to the distance sensor that shares the same name.
        //ensordistance3 = hardwareMap.get(DistanceSensor.class, "sensorcolordistanceleft");

        // eg: Set the drive motor directions:
        // Reverse the motor that runs backwards when connected directly to the battery
        // frontLeftMotor.setDirection(DcMotor.Direction.FORWARD); // Set to REVERSE if using AndyMark motors
        // frontRightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        // backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        // backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        //backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //rotateglyph.setDirection(DcMotor.Direction.FORWARD);
        //rotateglyph.setDirection(DcMotor.Direction.REVERSE);
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {

        rotateglyph.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("INIT LOOP, Rotate Reset: ", rotateglyph.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {

        ENCODERROTATE0 = "YES";

        rotateglyph.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("START, Rotate Reset: ", rotateglyph.getCurrentPosition());
        telemetry.update();
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {
        //loop_count = loop_count + 1;

        telemetry.addData("Status: ", "Running: " + runtime.toString());
        telemetry.addData("Rotate Position at Start of Loop: ", rotateglyph.getCurrentPosition());
        telemetry.update();

        //telemetry.addData("Loop Count ", +loop_count);
        telemetry.addData("Front Left Ticks:  ", frontleftmotor.getCurrentPosition());
        telemetry.addData("Back Left Ticks:   ", backleftmotor.getCurrentPosition());
        telemetry.addData("Front Right Ticks: ", frontrightmotor.getCurrentPosition());
        telemetry.addData("Back Right Ticks:  ", backrightmotor.getCurrentPosition());

        //telemetry.addData("Motor Output: ", "Front Left" + backleftmotor.getPower());
        //telemetry.addData("Motor Output: ", "Front Right" + backrightmotor.getPower());
        //telemetry.addData("Distance2 (cm)", String.format(Locale.US, "%.02f", sensordistance2.getDistance(DistanceUnit.CM)));
        //telemetry.addData("Distance3 (cm)", String.format(Locale.US, "%.02f", sensordistance3.getDistance(DistanceUnit.CM)));

        // get a reference to our digitalTouch object.
        digitaltouch = hardwareMap.get(DigitalChannel.class, "sensordigital");
        // set the digital channel to input.
        digitaltouch.setMode(DigitalChannel.Mode.INPUT);

        //FRONT DISTANCE
        // get a reference to our digitalTouch object.
        digitaltouch = hardwareMap.get(DigitalChannel.class, "sensordigital");
        // set the digital channel to input.
        digitaltouch.setMode(DigitalChannel.Mode.INPUT);
        //
        //
        //
        //
        //
        //
        // GAMEPAD 2, AUX DRIVER
        //
        //
        //
        //
        //
        //
        // GAMEPAD 2, LEFT AND RIGHT BUMPERS, ROTATE GLYPH WITH ENCODER
        //
        //
        if (gamepad2.left_bumper) {
            telemetry.addData("LEFT BUMPER ENCODER 180 BEFORE, IN IF: ", rotateglyph.getCurrentPosition());

            rotateglyph.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotateglyph.setPower(rotateglyphspeed);            		//0.2
            rotateglyph.setTargetPosition(rotateglyphto180);        	//-835

            ENCODERROTATE0   = "NO";

            telemetry.addData("LEFT BUMPER ENCODER 180 AFTER, IN IF: ", rotateglyph.getCurrentPosition());
            telemetry.update();
        }

        if (gamepad2.right_bumper) {
            telemetry.addData("RIGHT BUMPER ENCODER 0 BEFORE, IN IF: ", rotateglyph.getCurrentPosition());

            rotateglyph.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rotateglyph.setPower(rotateglyphspeed);
            rotateglyph.setTargetPosition(rotateglyphto0);

            ENCODERROTATE0   = "YES";

            telemetry.addData("RIGHT BUMPER ENCODER 0 AFTER, IN IF: ", rotateglyph.getCurrentPosition());
            telemetry.update();
        }


        //
        //
        // GAMEPAD2 X B, BOTTOM FRONT GLYPH GRABBERS
        //
        //
        if (ENCODERROTATE0 == "YES") {     	//TOP IS TOP, BOTTOM IS BOTTOM
            telemetry.addData("IN GRABBER 0: ", rotateglyph.getCurrentPosition());
            telemetry.update();

            if (gamepad2.x) {            //CLOSE
                servo0.setPosition(Servo0Close);        //Right Close
                servo1.setPosition(Servo1Close);        //Left Close
            }
            if (gamepad2.b) {            //OPEN
                servo0.setPosition(Servo0Open);        //Right Open
                servo1.setPosition(Servo1Open);        //Left Open
            }

            if (gamepad2.dpad_left) {        //CLOSE
                servo4.setPosition(Servo0Close);        //Right Close
                servo5.setPosition(Servo1Close);        //Left Close
            }
            if (gamepad2.dpad_right) {        //OPEN
                servo4.setPosition(Servo0Open);        //Right Open
                servo5.setPosition(Servo1Open);        //Left Open
            }
        }
        else if (ENCODERROTATE0 == "NO") {	//TOP IS BOTTOM, BOTTOM IS TOP
            telemetry.addData("IN GRABBER NOT 0: ", rotateglyph.getCurrentPosition());
            telemetry.update();

            if (gamepad2.x) {            //CLOSE
                servo4.setPosition(Servo0Close);        //Right Close
                servo5.setPosition(Servo1Close);        //Left Close
            }
            if (gamepad2.b) {            //OPEN
                servo4.setPosition(Servo0Open);        //Right Open
                servo5.setPosition(Servo1Open);        //Left Open
            }

            if (gamepad2.dpad_left) {        //CLOSE
                servo0.setPosition(Servo0Close);        //Right Close
                servo1.setPosition(Servo1Close);        //Left Close
            }
            if (gamepad2.dpad_right) {        //OPEN
                servo0.setPosition(Servo0Open);        //Right Open
                servo1.setPosition(Servo1Open);        //Left Open
            }

        }






/*
        //
        //
        // GAMEPAD2 X B, BOTTOM FRONT GLYPH GRABBERS
        //
        //
        if (gamepad2.x) {            //CLOSE
            servo0.setPosition(Servo0Close);        //Right Close
            servo1.setPosition(Servo1Close);        //Left Close
        }
        if (gamepad2.b) {            //OPEN
            servo0.setPosition(Servo0Open);        //Right Open
            servo1.setPosition(Servo1Open);        //Left Open
        }
        //
        //
        // GAMEPAD2 DPAD_LEFT, DPAD_RIGHT, TOP FRONT GLYPH GRABBERS
        //
        //
        if (gamepad2.dpad_left) {        //CLOSE
            servo4.setPosition(Servo0Close);        //Right Close
            servo5.setPosition(Servo1Close);        //Left Close
        }
        if (gamepad2.dpad_right) {        //OPEN
            servo4.setPosition(Servo0Open);        //Right Open
            servo5.setPosition(Servo1Open);        //Left Open
        }
*/




        //
        //
        // GAMEPAD2 LEFT STICK, GLYPH LIFT MOTOR WITH BRAKING
        //
        //
        liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        liftmotor.setPower(-gamepad2.left_stick_y * liftmotormultiplier);
        //
        //
        // GAMEPAD2, RIGHT STICK, RELIC ARM UP DOWN WITH BRAKING
        //
        //
        relicUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        relicUp.setPower(-gamepad2.right_stick_y * relicUpmultiplier);
        //
        //
        // GAMEPAD2 Y A, RELIC CLAW OPEN AND CLOSE
        //
        //
        if (gamepad2.y) {
            servo3.setPosition(RelicOpen); //open RELIC
        } else if (gamepad2.a) {
            servo3.setPosition(RelicClose); //close RELIC
        }
        //
        //
        // GAMEPAD2, DPAD UP and DOWN, RELIC ARM IN OUT
        //
        //
        if (gamepad2.dpad_up) {
            relicOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicOut.setPower(1);

        } else if (gamepad2.dpad_down) {
            relicOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicOut.setPower(-1);

        } else {
            relicOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            relicOut.setPower(0);
        }



        //
        //
        //
        //
        //
        //
        // GAMEPAD 1 DRIVER
        //
        //
        //
        //
        //
        //
        // GAMEPAD1, DPAD UP, ENCODER MOVE ON BOARD, NEW TEST NEW TEST
        //
       /* if (gamepad1.dpad_up) {

            if (dpadcount1 == 1) {
                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                dpadcount1 = dpadcount1 + 1;
            }

            frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            frontleftmotor.setPower(0.6);
            frontrightmotor.setPower(0.6);
            backleftmotor.setPower(0.6);
            backrightmotor.setPower(0.6);

            frontleftmotor.setTargetPosition(800);
            backleftmotor.setTargetPosition(800);
            frontrightmotor.setTargetPosition(-800);
            backrightmotor.setTargetPosition(-800);

            telemetry.addData("DPAD UP ENCODER MOB, FrontLeftMotor : ", frontleftmotor.getCurrentPosition());
            telemetry.addData("DPAD UP ENCODER MOB, FrontRightMotor: ", frontrightmotor.getCurrentPosition());
            telemetry.addData("DPAD UP ENCODER MOB, BackLeftMotor  : ", backleftmotor.getCurrentPosition());
            telemetry.addData("DPAD UP ENCODER MOB, BackRightMotor : ", backrightmotor.getCurrentPosition());
            telemetry.update();
        } else telemetry.update();
        */

        // dpadcount1 = 1;   //RESET DPAD COUNT

        //
        //
        // GAMEPAD1, DPAD LEFT RIGHT, ROTATE GLYPH ARM MANUALLY
        //

        if (gamepad1.right_trigger >= .99) {
            telemetry.addData("Right Trigger ON", 0);
            telemetry.update();

            if (gamepad1.dpad_left) {
                rotateglyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateglyph.setPower(.3);

                telemetry.addData("left", .1);
                telemetry.update();
            }
            else if (gamepad1.dpad_right) {
                rotateglyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateglyph.setPower(-.3);

                telemetry.addData("right", -.1);
                telemetry.update();
            }
            else {
                rotateglyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateglyph.setPower(0);

                telemetry.addData("ELSE", -.1);
                telemetry.update();
            }
        }
        //
        // GAMEPAD1, JOYSTICK BUTTON, ROBOT WIGGLE, NEW TEST NEW TEST
        //
       /* if (gamepad1.left_stick_button) {
            frontleftmotor.setPower(.6);  //Pos
            backleftmotor.setPower(.6);
            frontrightmotor.setPower(.6);  //Neg
            backrightmotor.setPower(.6);

            frontleftmotor.setPower(-.6);  //Pos
            backleftmotor.setPower(-.6);
            frontrightmotor.setPower(-.6);  //Neg
            backrightmotor.setPower(-.6);

            //frontleftmotor.setPower(0);
            //backleftmotor.setPower(0);
            //frontrightmotor.setPower(0);
            //backrightmotor.setPower(0);
        }
        */
        //
        // REMOVED
        // GAMEPAD1, LEFT TRIGGER, ROTATE GLYPH COUNTERCLOCKWISE - REMOVED
        // REMOVED
        //
   /*
            if (gamepad1.left_trigger >= .99) {
                telemetry.addData("LEFT TRIGGER ON: ", gamepad1.left_trigger);
                telemetry.update();

                rotateglyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateglyph.setPower(0.1);
            }
            else if (gamepad1.left_trigger < .98 && gamepad1.left_trigger > .01) {
                telemetry.addData("LEFT TRIGGER OFF: ", gamepad1.left_trigger);
                rotateglyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateglyph.setPower(0);
            }
            //
            // REMOVED
            // GAMEPAD1, RIGHT TRIGGER, ROTATE GLYPH CLOCKWISE - REMOVED
            // REMOVED
            //

            if (gamepad1.right_trigger >= .99) {
                telemetry.addData("RIGHT TRIGGER ON: ", gamepad1.right_trigger);
                telemetry.update();

                rotateglyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateglyph.setPower(-0.1);
            }
            else if (gamepad1.right_trigger < .98 && gamepad1.right_trigger > .01) {
                telemetry.addData("RIGHT TRIGGER OFF: ", gamepad1.right_trigger);
                rotateglyph.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                rotateglyph.setPower(0);
            }
  */
        //
        //
        // GAMEPAD1, Y, RAISE COLOR ARM IF NECESSARY
        // GAMEPAD1, Y, USED TO RESET ENCODERS FROM CONTROLLER IF NECESSARY
        //
        //
        if (gamepad1.y) {
            telemetry.addData("RESET ENCODERS and RAISE COLOR ARM: ", "0");
            telemetry.update();

            servo2.setPosition(ColorUp);		//RAISE COLOR ARM IF NECESSARY
            servo6.setPosition(.5);			//CENTER COLOR ARM LEFT RIGHT IF NECESSARY

            rotateglyph.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            telemetry.addData("RESET rotate: ", rotateglyph.getCurrentPosition());
            telemetry.addData("RESET fl: ", frontleftmotor.getCurrentPosition());
            telemetry.addData("RESET fr: ", frontrightmotor.getCurrentPosition());
            telemetry.addData("RESET bl: ", backleftmotor.getCurrentPosition());
            telemetry.addData("RESET br: ", backrightmotor.getCurrentPosition());
            telemetry.update();
        }
        //
        //
        // GAMEPAD1, RIGHT BUMPER, LEFT BUMPER, ROBOT SPEED CONTROL
        //
        // ROBOT SPEED MULTIPLIERS
        //
        if (gamepad1.right_bumper) {
            speedreduction = speedfast;		//was 1.5;
            telemetry.addData("Right Bumper, FAST BUTTON: ", speedreduction);
            telemetry.update();
        }
        if (gamepad1.left_bumper) {
            speedreduction = speedslow;		//was 4;
            telemetry.addData("Left Bumper, SLOW BUTTON: ", speedreduction);
            telemetry.update();
        }
        else {
            speedreduction = speednormal;		//was 1.5
        }
        //
        //
        // GAMEPAD1, LEFT AND RIGHT JOYSTICKS, MECANUM MATH
        //
        //
        frontleftspeed  = ((-gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x /1.0))/speedreduction);

        backleftspeed   = ((-gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x /1.0))/speedreduction);

        frontrightspeed = (( gamepad1.left_stick_y + gamepad1.left_stick_x + (gamepad1.right_stick_x /1.0))/speedreduction);

        backrightspeed  = (( gamepad1.left_stick_y + gamepad1.left_stick_x - (gamepad1.right_stick_x /1.0))/speedreduction);

        /*
        if(frontleftspeed > 1) {
            frontleftmotor.setPower(1);
        }
        if(backleftspeed > 1) {
            backleftmotor.setPower(1);
        }
        if(frontrightspeed > 1) {
            frontrightmotor.setPower(1);
        }
        if(backleftspeed > 1) {
            backleftmotor.setPower(1);
        }
        */


        telemetry.addData("FRONT LEFT  MATH:  ", frontleftspeed);
        telemetry.addData("BACK  LEFT  MATH:  ", backleftspeed);
        telemetry.addData("FRONT RIGHT MATH:  ", frontrightspeed);
        telemetry.addData("BACK  RIGHT MATH:  ", backrightspeed);

        frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        frontleftmotor.setPower(frontleftspeed);
        backleftmotor.setPower(backleftspeed);
        frontrightmotor.setPower(frontrightspeed);
        backrightmotor.setPower(backrightspeed);
        //
        //
        // GAMEPAD1 B, BRAKE AND HOLD WITH ENCODER
        //
        //
        if (gamepad1.b) {
            telemetry.addData("IN ENCODER BRAKE: ", frontleftmotor.getCurrentPosition());

            //telemetry.addData("ENCODER BRAKE Ticks fl: ", frontleftmotor.getCurrentPosition());
            //telemetry.addData("ENCODER BRAKE Ticks fr: ", frontrightmotor.getCurrentPosition());
            //telemetry.addData("ENCODER BRAKE Ticks bl: ", backleftmotor.getCurrentPosition());
            //telemetry.addData("ENCODER BRAKE Ticks br: ", backrightmotor.getCurrentPosition());

            frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftmotor.setPower(0.3);
            frontrightmotor.setPower(0.3);
            backleftmotor.setPower(0.3);
            backrightmotor.setPower(0.3);

            //telemetry.addData("COUNT1 BEFORE IF: ", count1);
            if (count1 == 1){
                //GRAB AND SAVE ONLY ONCE, THE POSITION YOU WANT TO HOLD
                //telemetry.addData("IN COUNT1 IF: ", count1);
                getcurposfrontleft = frontleftmotor.getCurrentPosition();
                getcurposfrontright = frontrightmotor.getCurrentPosition();
                getcurposbackleft = backleftmotor.getCurrentPosition();
                getcurposbackright = backrightmotor.getCurrentPosition();

                count1 = count1 + 1;
            }

            //telemetry.addData("COUNT1 AFTER IF: ", count1);
            telemetry.addData("BRAKE Pos fl: ", getcurposfrontleft);
            telemetry.addData("BRAKE Pos fr: ", getcurposfrontright);
            telemetry.addData("BRAKE Pos bl: ", getcurposbackleft);
            telemetry.addData("BRAKE Pos br: ", getcurposbackright);
            telemetry.update();

            frontleftmotor.setTargetPosition(getcurposfrontleft);
            frontrightmotor.setTargetPosition(getcurposfrontright);
            backleftmotor.setTargetPosition(getcurposbackleft);
            backrightmotor.setTargetPosition(getcurposbackright);
        }
        else {
            frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            count1 = 1;  //Reset Count for another brake attempt
        }
        //
        //
        // GAMEPAD 1 X, STOP/BRAKE/FLOAT
        //
        //
        if (gamepad1.x) {
            telemetry.addData("ZERO POWER BRAKE l: ", frontleftmotor.getCurrentPosition());
            telemetry.addData("ZERO POWER BRAKE r: ", frontrightmotor.getCurrentPosition());
            telemetry.update();

            frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

            //relicUp.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);  //WAS THIS ADDED SAT?
            //relicOut.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); //WAS THIS ADDED SAT??

            frontleftmotor.setPower(0.00);
            frontrightmotor.setPower(0.00);
            backleftmotor.setPower(0.00);
            backrightmotor.setPower(0.00);
        }
        else {
            //telemetry.addData("ZERO POWER FLOAT l: ", frontleftmotor.getCurrentPosition());
            //telemetry.addData("ZERO POWER FLOAT r: ", frontrightmotor.getCurrentPosition());

            frontleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            frontrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backleftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            backrightmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }
        //
    }

    @Override
    public void stop() {
    }
}