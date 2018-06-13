/**
 * Created by Mach Speed Programming on 10/31/2017.
 */

// AUTO RED BACK   AUTONOMOUS BY TIME WITH COLOR, DISTANCE and TOUCH CODE
//
// CHANGES
//
// 24-MAR-2018 - Modified Code for Mecanum Wheel Effect
// 08-FEB-2018 - Backup and Push Steps Commented Out
// 02-FEB-2018 - Made sure to shut off Lift Motor correctly
//               Adjusted times to match other Autos
// 15-JAN-2018 - Added Picto Code
//		Added Reset Speed to compensate for over rotation
//		Changed STOP to .1 to speed up Auto
//
//
package org.firstinspires.ftc.teamcode;// AUTONOMOUS BY TIME WITH COLOR, DISTANCE and TOUCH CODE

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Servo;



//START PICTO IMPORTS
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
//END PICTO IMPORTS


@Autonomous(name = "AutoRedBack", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list

public class AutoRedBack extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontleftmotor;
    DcMotor frontrightmotor;
    DcMotor backleftmotor;
    DcMotor backrightmotor;

    DcMotor liftmotor;
    DcMotor rotateglyph;

    Servo servo0;  //Grabber
    Servo servo1;  //Grabber
    Servo servo2;  //Sensor Arm
    Servo servo6;  //Sensor Move

    DigitalChannel digitaltouch;  // Hardware Device Object

    ColorSensor sensorcolor;
    DistanceSensor sensordistance;

    Boolean Isblue;

    int ColorBlue = 0;
    int pictocount = 1;         //this is for PICTO

    //RIGHT GRABBER ARM
    static final double Servo0Open = .65;    //Right Open        1
    static final double Servo0Close = .45;     //Right Close     0

    //LEFT GRABBER ARM
    static final double Servo1Close = .55;    //Left Close       1
    static final double Servo1Open = .35;     //Left Open        0

    //FULL OPEN GRABBER ARM
    static final double Servo0FullOpen = .95;  		//Right Full Open
    static final double Servo1FullOpen = .05;  	//Left Full Open

    //FULL CLOSE GRABBER ARM
    static final double Servo0FullClose = .3;  		//Right super close
    static final double Servo1FullClose = .85;  	//Left super close


    //START AND END PARMS
    double StartTime    = 0.0;
    double EndTime      = 0.0;

    //BELOW VALUES IN SECONDS
    //double CloseArms            = 1.0;      //Step 1

    double DropColorArm         = 1.0;      //
    double ReadColor            = 1.0;      //

    double GlyphMotorPowerUp 	=  .4;	    //RAISE GLYPH ARM was .6, .4 a low as it can go
    double GlyphMotorPowerDown	= -.3;	    //LOWER GLYPH ARM, BUT NOT COMPLETELY

    double KnockOffBall         = 1.5;      //

    double ResetMove            = 1.0;      //

    double Stop                 = 0.15;      //
    double RaiseColorArm        = 1.0;      //

    double StopEncoders         = 0.15;      //
    //
    //
    // 41 Ticks equals about 1 inch
    // 38.5 Ticks NOW Equals about 1 inch with 4 drive motors
    //
    //
    double MoveOffBoard         = 2.5;        	//
    int MoveOffBoardClicksLeft  =  1001;     	//was 995
    int MoveOffBoardClicksRight = -1001;       	//
    double MOBPower 		    = .3;


    double CrabLeft		        = 4.0;
    double CrabLeftPower    	=  .4;		//worked at .4

    int CrabLeftLeftFL		    = -1200;		//was -1142, worked 3 times at -1200
    int CrabLeftLeftBL          =  1300; 		//was 1242, worked 3 times at 1300
    int CrabLeftLeftFR          = -1200;
    int CrabLeftLeftBR          =  1300;

    int CrabLeftCenterFL	= -773; //732
    int CrabLeftCenterBL	=  873; //832
    int CrabLeftCenterFR	= -773;
    int CrabLeftCenterBR	=  873;

    int CrabLeftRightFL		= -383;
    int CrabLeftRightBL		=  483;
    int CrabLeftRightFR		= -383;
    int CrabLeftRightBR		=  483;



    //MTG = move to glyph
    double MoveParallel         = 4.0;        	//NEW PARM, to better control movement times, was moveoffboard 3 sec
    double MoveParallelPower    =  .3;

    //MECANUM CHASSIS
    int MTGClicksLeftLEFT      	=  993;     	//LEFT //was 953
    int MTGClicksRightLEFT     	= -993;     	//LEFT
    int MTGClicksLeftCENTER    	=  600;     	//CENTER was 513
    int MTGClicksRightCENTER   	= -600;     	//CENTER was -513
    int MTGClicksLeftRIGHT     	=  254;     	//RIGHT was good at 254 world
    int MTGClicksRightRIGHT    	= -254;     	//RIGHT


    //STATE VALUES ORIG CHASSIS
    //int MTGClicksLeftLEFT      	= -871;     	//LEFT
    //int MTGClicksRightLEFT     	=  871;     	//LEFT
    //int MTGClicksLeftCENTER    	= -513;     	//CENTER
    //int MTGClicksRightCENTER   	=  513;     	//CENTER
    //int MTGClicksLeftRIGHT     	= -172;     	//RIGHT
    //int MTGClicksRightRIGHT    	=  172;     	//RIGHT

    //START PICTO PARAMS
    double Spin                 = 3.5;  		// SPIN 1
    int Spin90leftLEFT          = -1010;     //was 939
    int Spin90rightLEFT         = -1010;

    int Spin90leftRIGHT         = 1010;  	//SPIN 2, Seeing 554, 549 in telemetry 539
    int Spin90rightRIGHT        = 1010;     //was 939
    double SpinPower 		    = .7;



    //END PICTO PARAMS

    double MoveToWall           = 2.0;       	//
    int MoveToWallClicksLeft    = 500;     	//Forward, Add 1 inch was 415
    int MoveToWallClicksRight   = -500;      	//Forward
    double MoveToWallPower 	    = .3;

    double DropGlyph            = 1.0;      	//DROP GLYPH, NO MOTOR MOVEMENT

    //double BackUp                 = 0.0;        	//REMOVED
    //int BackUpClicksLeft          =  205;      	//
    //int BackUpClicksRight         = -205;     	//
    //double BackupPower 		    = .4;

    //double PushGlyph              = 2.0;        	//Was .5
    //int PushGlyphClicksLeft       = -418;     	//Forward, Seeing -410, 412 in telemetry
    //int PushGlyphClicksRight      =  418;      	//Forward
    //double PushPower 		        = .4;

    double BackUpFinal              = 2.0;        	//Was .05
    int BackUpFinalClicksLeft       =  -205;      	//Backward
    int BackUpFinalClicksRight      =  205;     	//Backward
    double BackUpFinalPower 	    = .3;

    //
    // SERVO CODE
    //
    // LOWER COLOR ARM
    // .75 is about perfect position lowering color arm
    // .55 is probably too high off the ground
    static final double MAX_POS     =  0;     // Maximum rotational position LOWER .75

    // RAISE COLOR ARM
    // .2 is about perfect position for raising color arm
    // .1 raises it too much
    // .3 arm ends up way too low in the raised position
    static final double MIN_POS     =  0.61;     // Minimum rotational position RAISE 0.2

    double  positionx = (MAX_POS - MIN_POS) / 2;

    //Color Arm Sideways
    static final double Servo6Left 	    = .68; //was .65
    static final double Servo6Right 	= .32; //was .45
    static final double Servo6Center 	= .5;   //CENTER



    //START PICTO INIT
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    String machspeedpicto = "NONE";
    //END PICTO INIT


    @Override
    public void runOpMode() {


        frontleftmotor = hardwareMap.dcMotor.get("frontleftdrive");
        frontrightmotor = hardwareMap.dcMotor.get("frontrightdrive");
        backleftmotor = hardwareMap.dcMotor.get("backleftdrive");
        backrightmotor = hardwareMap.dcMotor.get("backrightdrive");

        // get a reference to our digitalTouch object.

        digitaltouch = hardwareMap.get(DigitalChannel.class, "sensordigital");
        // set the digital channel to input.
        digitaltouch.setMode(DigitalChannel.Mode.INPUT);

        servo0 = hardwareMap.servo.get("servo0");
        servo1 = hardwareMap.servo.get("servo1");
        servo2 = hardwareMap.servo.get("servo2");
        servo6 = hardwareMap.servo.get("servo6");

        liftmotor = hardwareMap.dcMotor.get("liftmotor");

        rotateglyph  = hardwareMap.dcMotor.get("rotateglyph");

        // get a reference to the color sensor.
        sensorcolor = hardwareMap.get(ColorSensor.class, "sensorcolordistance");
        // get a reference to the distance sensor that shares the same name.
        sensordistance = hardwareMap.get(DistanceSensor.class, "sensorcolordistance");

        // hsvValues is an array that will hold the hue, saturation, and value information.
        float hsvValues[] = {0F, 0F, 0F};

        // values is a reference to the hsvValues array.
        final float values[] = hsvValues;

        // sometimes it helps to multiply the raw RGB values with a scale factor
        // to amplify/attentuate the measured values.
        final double SCALE_FACTOR = 255;

        // get a reference to the RelativeLayout so we can change the background
        // color of the Robot Controller app to match the hue detected by the RGB sensor.
        int relativeLayoutId = hardwareMap.appContext.getResources().getIdentifier("RelativeLayout", "id", hardwareMap.appContext.getPackageName());
        final View relativeLayout = ((Activity) hardwareMap.appContext).findViewById(relativeLayoutId);


        //START PICTO
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);
        parameters.vuforiaLicenseKey = "AbM6uEj/////AAAAmeqhv80gaUp9rQCz+WZW1eRPJsVnwHUVATdeG+vmO6Ptp5EOyi+zdBId4Em/tfAVLCFxuLZMmPDJ4t7V9z0IOWNQJRK+83tXUlS0Bwtnv9woSdPcROg0nSUu+EeYqe68GtQc5II9Bi/q2/nKG8TiN/hzwDI2H3OJFy0fND2rLBf/A7pgxcdIMu0ITBx+1lZLgdNFGfyU/IvPxqHaWPUVNFH6QG7HZfyUYSkcRLJLmrDm/zOtA454GZDiTZrGusoqEPlJan9CrJjWTisDXWg8NaHG7j2A5V2gDugq4WVnfAPDFUyzftbmK5oNH9lmT4d3+ouIssOn1SqxjuvF+9Q4MAdcz3rfXwpSjvgBfifRNIY2";
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        this.vuforia = ClassFactory.createVuforiaLocalizer(parameters);
        VuforiaTrackables relicTrackables = this.vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicTemplate = relicTrackables.get(0);
        relicTemplate.setName("relicVuMarkTemplate"); // can help in debugging; otherwise not necessary

        telemetry.addData(">", "Press Play to start");
        telemetry.update();
        //END PICTO


        // wait for the start button to be pressed
        waitForStart();


        // LOOP LOOP LOOP
        // loop and read the RGB and distance data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.

        runtime.reset();

        frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //START PICTO
        relicTrackables.activate();
        //END PICTO


        while (opModeIsActive()) {

            // convert the RGB values to HSV values.
            // multiply by the SCALE_FACTOR.
            // then cast it back to int (SCALE_FACTOR is a double)
            Color.RGBToHSV((int) (sensorcolor.red() * SCALE_FACTOR),
                    (int) (sensorcolor.green() * SCALE_FACTOR),
                    (int) (sensorcolor.blue() * SCALE_FACTOR),
                    hsvValues);

            // Send the info back to driver station using telemetry function.

            telemetry.addData("START AUTO RED BACK: ", 0);

            //telemetry.addData("Motor Output ", "Back Left" + backleftmotor.getPower());
            //telemetry.addData("Motor Output ", "Back Right" + backrightmotor.getPower());
            //telemetry.addData("Distance (cm)", String.format(Locale.US, "%.02f", sensordistance.getDistance(DistanceUnit.CM)));
            //telemetry.addData("Hue", hsvValues[0]);


            //START PICTO
            //
            //if (pictocount <= 2) {
            //    telemetry.addData("In PictoCount = ",pictocount);
            RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.from(relicTemplate);
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {

                telemetry.addData("VuMark Chuck1 ", "%s visible", vuMark);
                //telemetry.addData("Location ", vuMark);
                //machspeedpicto = vuMark;

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    machspeedpicto = "LEFT";
                }
                else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    machspeedpicto = "CENTER";
                }
                else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    machspeedpicto = "RIGHT";
                }
                else
                    machspeedpicto = "CENTER";  {
                }

                telemetry.addData("SAVED PICTO: ",machspeedpicto);

                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) relicTemplate.getListener()).getPose();
                //telemetry.addData("Pose", format(pose));

                if (pose != null) {
                    VectorF trans = pose.getTranslation();
                    Orientation rot = Orientation.getOrientation(pose, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);

                    // Extract the X, Y, and Z components of the offset of the target relative to the robot
                    double tX = trans.get(0);
                    double tY = trans.get(1);
                    double tZ = trans.get(2);

                    // Extract the rotational components of the target relative to the robot
                    double rX = rot.firstAngle;
                    double rY = rot.secondAngle;
                    double rZ = rot.thirdAngle;
                }
                //telemetry.addData("Before PictoCount 2 = ",pictocount);
                pictocount = pictocount + 1;
                //telemetry.addData("After PictoCount 2 = ",pictocount);
            } else {
                telemetry.addData("Else VuMark ", "not visible");
                //telemetry.addData("Before PictoCount 3 = ",pictocount);
                pictocount = pictocount + 1;
            }
            //    telemetry.addData("Else PictoCount 4 = ",pictocount);

            //}

            telemetry.update();

            //if (vuMark == RelicRecoveryVuMark.CENTER) {
            //    telemetry.addData("SAVED CENTER ",vuMark);
            //
            //}
            //END PICTO
            //
            //
            //
            //
            //
            //
            //
            //START, GRAP GLYPH IN FRONT, DROP COLOR ARM
            //
            StartTime = 0;
            EndTime = StartTime + DropColorArm;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Start Reset Color, Grab Glyph: ", "1");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                //SET GLYPH ROTATE TO ZERO
                rotateglyph.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //GRAB GLYPH //
                servo0.setPosition(Servo0Close);  	//Clamp Glyph
                servo1.setPosition(Servo1Close); 		//Clamp Glyph

                //CENTER COLOR LEFT RIGHT MOVEMENT
                servo6.setPosition(Servo6Center);

                //DROP COLOR ARM
                servo2.setPosition(MIN_POS);

                telemetry.addData("DROP ARM: ", MIN_POS);
                telemetry.update();
            }
            //
            //READ COLOR, STOP GLYPH LIFT
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + ReadColor;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Read Color, Stop Glyph Arm: ", "2");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);


                // ASSUME WE ARE RED TEAM
                // ASSUME SENSOR FACING BACKWARDS ON ROBOT ON RIGHT SIDE
                // CHECK IF STATEMENT FOR BLUE ONLY
                if (hsvValues[0] >= 130 && hsvValues[0] <= 250) {
                    //add variable
                    ColorBlue = 1;
                    telemetry.addData("BALL COLOR BLUE: ", sensorcolor.blue());
                    telemetry.addData("Hue", hsvValues[0]);
                } else {
                    telemetry.addData("BALL COLOR RED: ", sensorcolor.red());
                    telemetry.addData("Hue", hsvValues[0]);
                    ColorBlue = 0;
                }
                telemetry.update();
            }
            //
            // KNOCK OFF BALL
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + KnockOffBall;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Knock Off Ball: ", "3");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                if (ColorBlue == 1) {
                    telemetry.addData("Knock Off BLUE Ball: ", "Running: " + runtime.toString());

                    servo6.setPosition(Servo6Right);
                    telemetry.addData("Servo6Right: ", + Servo6Right);
                } else {
                    telemetry.addData("Knock off RED Ball: ", "Running: " + runtime.toString());

                    servo6.setPosition(Servo6Left);
                    telemetry.addData("Servo6Left: ", + Servo6Left);
                }
            }
            //
            //RAISE COLOR ARM, RAISE GLYPH ARM
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + RaiseColorArm;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Raise Color Arm: ", "4");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                //RAISE COLOR ARM
                servo2.setPosition(MAX_POS);		//RAISE COLOR ARM
                servo6.setPosition(Servo6Center);	//CENTER COLOR ARM

                liftmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                liftmotor.setPower(GlyphMotorPowerUp); 	//RAISE GLYPH ARM
            }
            //
            //STOP BEFORE ENCODERS
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Stop;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("STOP After Reset: ", "5");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                liftmotor.setPower(0);  		//STOP GLYPH ARM

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //
            //
            //
            //
            //ENCODER MOVEMENT 5 SECONDS
            //MOVE OFF BOARD
            //
            //
            //
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + MoveOffBoard;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Move Off Board: ", "6");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(MOBPower);
                backleftmotor.setPower(MOBPower);
                frontrightmotor.setPower(MOBPower);
                backrightmotor.setPower(MOBPower);

                frontleftmotor.setTargetPosition(MoveOffBoardClicksLeft);
                backleftmotor.setTargetPosition(MoveOffBoardClicksLeft);
                frontrightmotor.setTargetPosition(MoveOffBoardClicksRight);
                backrightmotor.setTargetPosition(MoveOffBoardClicksRight);

                //telemetry.addData("MOVE OFF Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE OFF Right Ticks: ", backrightmotor.getCurrentPosition());
            }
            //
            //STOP
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Stop;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("STOP After Reset: ", "7");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //SPIN 1
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Spin;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Spin 1: ", "11");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(SpinPower);
                backleftmotor.setPower(SpinPower);
                frontrightmotor.setPower(SpinPower);
                backrightmotor.setPower(SpinPower);

                frontleftmotor.setTargetPosition(Spin90leftLEFT);
                backleftmotor.setTargetPosition(Spin90leftLEFT);
                frontrightmotor.setTargetPosition(Spin90rightLEFT);
                backrightmotor.setTargetPosition(Spin90rightLEFT);

                //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
            }

            //
            //STOP
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Stop;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("STOP After Reset: ", "12");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //
            // CRAB TO THE LEFT TEST    REMOVED
            //
/*
	    StartTime = EndTime + .1;
            EndTime = StartTime + 1.5;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {

                frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

                telemetry.addData("CRAB BEFORE FL Ticks: ", frontleftmotor.getCurrentPosition());
                telemetry.addData("CRAB BEFORE BL Ticks: ", backleftmotor.getCurrentPosition());
                telemetry.addData("CRAB BEFORE FR Ticks: ", frontrightmotor.getCurrentPosition());
                telemetry.addData("CRAB BEFORE BR Ticks: ", backrightmotor.getCurrentPosition());
                telemetry.update();

                //CRAB LEFT
                frontleftmotor.setPower(-.6);   //Negative
                backleftmotor.setPower(.6);     //Positive
                frontrightmotor.setPower(-.6);  //Negative
                backrightmotor.setPower(.6);	//Positive

                telemetry.addData("CRAB AFTER FL Ticks: ", frontleftmotor.getCurrentPosition());
                telemetry.addData("CRAB AFTER BL Ticks: ", backleftmotor.getCurrentPosition());
                telemetry.addData("CRAB AFTER FR Ticks: ", frontrightmotor.getCurrentPosition());
                telemetry.addData("CRAB AFTER BR Ticks: ", backrightmotor.getCurrentPosition());
                telemetry.update();
            }
*/
            //
            //
            // CRAB TO THE LEFT LIVE!!!!!!!
            //
/*
            StartTime = EndTime + .1;
            EndTime = StartTime + CrabLeft;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Move Parallel to Cryptobox: ", "13");
                telemetry.addData("StartTime   ", StartTime);
                telemetry.addData("EndTime     ", EndTime);
                //telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                //telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(CrabLeftPower);
                frontrightmotor.setPower(CrabLeftPower);
                backleftmotor.setPower(CrabLeftPower);
                backrightmotor.setPower(CrabLeftPower);

                if (machspeedpicto == "LEFT") {
                    frontleftmotor.setTargetPosition(CrabLeftLeftFL);
                    backleftmotor.setTargetPosition(CrabLeftLeftBL);
                    frontrightmotor.setTargetPosition(CrabLeftLeftFR);
                    backrightmotor.setTargetPosition(CrabLeftLeftBR);		//was BL!!!
                    telemetry.addData("CRAB LEFT PICTO: ", backleftmotor.getCurrentPosition());
                }
                else if (machspeedpicto == "CENTER") {
                    frontleftmotor.setTargetPosition(CrabLeftCenterFL);
                    backleftmotor.setTargetPosition(CrabLeftCenterBL);
                    frontrightmotor.setTargetPosition(CrabLeftCenterFR);
                    backrightmotor.setTargetPosition(CrabLeftCenterBR);
                    telemetry.addData("CRAB CENTER PICTO: ", backleftmotor.getCurrentPosition());
                }
                else if (machspeedpicto == "RIGHT") {
                    frontleftmotor.setTargetPosition(CrabLeftRightFL);
                    backleftmotor.setTargetPosition(CrabLeftRightBL);
                    frontrightmotor.setTargetPosition(CrabLeftRightFR);
                    backrightmotor.setTargetPosition(CrabLeftRightBR);
                    telemetry.addData("CRAB RIGHT PICTO: ", backleftmotor.getCurrentPosition());
                }
                else {
                    frontleftmotor.setTargetPosition(CrabLeftCenterFL);
                    backleftmotor.setTargetPosition(CrabLeftCenterBL);
                    frontrightmotor.setTargetPosition(CrabLeftCenterFR);
                    backrightmotor.setTargetPosition(CrabLeftCenterBR);
                    telemetry.addData("PICTO FAIL, CRAB DEFAULT CENTER: ", backleftmotor.getCurrentPosition());
                }

                telemetry.addData("CRAB FL Ticks: ", frontleftmotor.getCurrentPosition());
                telemetry.addData("CRAB FR Ticks: ", frontrightmotor.getCurrentPosition());
                telemetry.addData("CRAB BL Ticks: ", backleftmotor.getCurrentPosition());
                telemetry.addData("CRAB BR Ticks: ", backrightmotor.getCurrentPosition());

                telemetry.update();
            }
*/
            //
            //
            //MOVE TO GLYPH SPOT
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + MoveParallel;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Move Parallel to Cryptobox: ", "13");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                //frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                //frontleftmotor.setPower(0.3);
                //frontrightmotor.setPower(0.3);
                backleftmotor.setPower(0.3);
                backrightmotor.setPower(0.3);

                if (machspeedpicto == "LEFT") {
                    backleftmotor.setTargetPosition(MTGClicksLeftLEFT);
                    backrightmotor.setTargetPosition(MTGClicksRightLEFT);
                    telemetry.addData("LEFT PICTO: ", backleftmotor.getCurrentPosition());
                }
                else if (machspeedpicto == "CENTER") {
                    backleftmotor.setTargetPosition(MTGClicksLeftCENTER);
                    backrightmotor.setTargetPosition(MTGClicksRightCENTER);
                    telemetry.addData("CENTER PICTO: ", backleftmotor.getCurrentPosition());
                }
                else if (machspeedpicto == "RIGHT") {
                    backleftmotor.setTargetPosition(MTGClicksLeftRIGHT);
                    backrightmotor.setTargetPosition(MTGClicksRightRIGHT);
                    telemetry.addData("RIGHT PICTO: ", backleftmotor.getCurrentPosition());

                }
                else {
                    backleftmotor.setTargetPosition(MTGClicksLeftCENTER);
                    backrightmotor.setTargetPosition(MTGClicksRightCENTER);
                    telemetry.addData("PICTO FAIL, DEFAULT CENTER: ", backleftmotor.getCurrentPosition());

                }
                //frontleftmotor.setTargetPosition(MoveOffBoardClicksLeft);   	// -1025 FORWARD about 14.5 inches
                //frontrightmotor.setTargetPosition(MoveOffBoardClicksRight);       // 1025 FORWARD
                // backleftmotor.setTargetPosition(MoveOffBoardClicksLeft);   	// -1025 FORWARD about 14.5 inches
                //backrightmotor.setTargetPosition(MoveOffBoardClicksRight);       // 1025 FORWARD

                //telemetry.addData("MOVE OFF Left Ticks:  ", frontleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE OFF Right Ticks: ", frontrightmotor.getCurrentPosition());
                //telemetry.addData("MOVE TO Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE TO Right Ticks: ", backrightmotor.getCurrentPosition());

                telemetry.update();
            }

            //
            //STOP AND RESET, LOWER GLYPH ARM
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset, Lower Glyph Arm: ", "14");

                //liftmotor.setPower(GlyphMotorPowerDown);   //LOWER GLYPH ARM

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //LOWER
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + .5;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset, Lower Glyph Arm: ", "14");

                liftmotor.setPower(-.18);   //LOWER GLYPH ARM

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //STOP AND RESET,
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset, Lower Glyph Arm: ", "14");

                liftmotor.setPower(0);   //LOWER GLYPH ARM

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //
            //SPIN 2, STOP GLYPH ARM
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Spin;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Spin 2: ", "15");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                liftmotor.setPower(0);   //STOP GLYPH ARM

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(SpinPower);
                backleftmotor.setPower(SpinPower);
                frontrightmotor.setPower(SpinPower);
                backrightmotor.setPower(SpinPower);

                frontleftmotor.setTargetPosition(Spin90leftRIGHT);
                backleftmotor.setTargetPosition(Spin90leftRIGHT);
                frontrightmotor.setTargetPosition(Spin90rightRIGHT);
                backrightmotor.setTargetPosition(Spin90rightRIGHT);

                //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
            }

            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "16");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //
            //MOVE TO WALL
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + MoveToWall;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {

                liftmotor.setPower(GlyphMotorPowerDown);   //LOWER GLYPH ARM

                telemetry.addData("Encoder, Move to Wall: ", "17");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(MoveToWallPower);
                backleftmotor.setPower(MoveToWallPower);
                frontrightmotor.setPower(MoveToWallPower);
                backrightmotor.setPower(MoveToWallPower);

                frontleftmotor.setTargetPosition(MoveToWallClicksLeft);
                backleftmotor.setTargetPosition(MoveToWallClicksLeft);
                frontrightmotor.setTargetPosition(MoveToWallClicksRight);
                backrightmotor.setTargetPosition(MoveToWallClicksRight);

                //telemetry.addData("MOVE TO Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE TO Right Ticks: ", backrightmotor.getCurrentPosition());
            }
            //
            //STOP RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "18");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //DROP GLYPH, NO MOTOR MOVEMENT
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + DropGlyph;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Drop Glyph: ", "19");

                servo0.setPosition(Servo0Open); 		//DROP GLYPH
                servo1.setPosition(Servo1Open); 		//DROP GLYPH
            }

/*
            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "22");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            //
            //PUSH GLYPH
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + PushGlyph;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Push Glyph: ", "23");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setPower(0.4);
                backrightmotor.setPower(0.4);

                backleftmotor.setTargetPosition(PushGlyphClicksLeft); //FORWARD about 14.5 inches
                backrightmotor.setTargetPosition(PushGlyphClicksRight); //FORWARD
            }
            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "24");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
*/
            //
            //BACK UP AFTER PUSH
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + BackUpFinal;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Backup Final: ", "25");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(BackUpFinalPower);
                backleftmotor.setPower(BackUpFinalPower);
                frontrightmotor.setPower(BackUpFinalPower);
                backrightmotor.setPower(BackUpFinalPower);

                frontleftmotor.setTargetPosition(BackUpFinalClicksLeft); 	//BACKWARD
                backleftmotor.setTargetPosition(BackUpFinalClicksLeft); 	//BACKWARD
                frontrightmotor.setTargetPosition(BackUpFinalClicksRight); 	//BACKWARD
                backrightmotor.setTargetPosition(BackUpFinalClicksRight); 	//BACKWARD
            }
            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "26");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
           /* //
            //SPIN TO PILE
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Spin;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Spin To Pile: ", "27");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(SpinPower);
                backleftmotor.setPower(SpinPower);
                frontrightmotor.setPower(SpinPower);
                backrightmotor.setPower(SpinPower);

                frontleftmotor.setTargetPosition(Spin90leftLEFT);
                backleftmotor.setTargetPosition(Spin90leftLEFT);
                frontrightmotor.setTargetPosition(Spin90rightLEFT);
                backrightmotor.setTargetPosition(Spin90rightLEFT);

                //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());

            }
            //
            //STOP AND RESET FINAL
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Final Stop: ", "28");

                rotateglyph.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateglyph.setPower(.3);
                rotateglyph.setTargetPosition(0);

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }*/
            //
            //
            // change the background color to match the color detected by the RGB sensor.
            // pass a reference to the hue, saturation, and value array as an argument
            // to the HSVToColor method.

            relativeLayout.post(new Runnable() {
                public void run() {
                    relativeLayout.setBackgroundColor(Color.HSVToColor(0xff, values));
                }
            });

            telemetry.update();		//SEND TELEMETRY TO PHONE
        }

        // Set the panel back to the default color
        relativeLayout.post(new Runnable() {
            public void run() {
                relativeLayout.setBackgroundColor(Color.WHITE);
            }
        });
    }
    //START PICTO "FORMAT MATRIX"
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    //END PICTO "FORMAT MATRIX"
}


