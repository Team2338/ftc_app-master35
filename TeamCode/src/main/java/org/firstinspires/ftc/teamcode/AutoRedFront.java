/**
 * Created by Mach Speed Programming on 10/31/2017.
 */

// AUTO RED FRONT   AUTONOMOUS BY TIME WITH COLOR, DISTANCE and TOUCH CODE
//
// CHANGES
//
// 24-MAR-2018 - Modified Code for Mecanum Wheel Effect
// 02-FEB-2018 - Made sure to shut off Lift Motor correctly
//               Adjusted times to match other Autos
// 15-JAN-2018 - Added Picto Code
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


@Autonomous(name = "AutoRedFront", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list

public class AutoRedFront extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    DcMotor frontleftmotor;
    DcMotor backleftmotor;
    DcMotor frontrightmotor;
    DcMotor backrightmotor;

    DcMotor liftmotor;

    DcMotor rotateglyph;

    Servo servo0;  //Grabber
    Servo servo1;  //Grabber
    Servo servo2;  //Sensor Arm
    Servo servo6;  //Sensor Arm Sideways

    DigitalChannel digitaltouch;  // Hardware Device Object

    ColorSensor sensorcolor;
    DistanceSensor sensordistance;



    String SPINTOPILE 		= "NO";  //YES = SPIN ROBOT TO FACE PILE OF GLYPHS, NO = POINT AT CRYPTOBOX
    String GOFORSECONDGLYPH 	= "NO";  //YES = GO FOR SECOND GLYPH, NO = REGULAR MODE


    int ColorBlue  = 0;
    int pictocount = 1;         //this is for PICTO

    //RIGHT GRABBER ARM
    static final double Servo0Open 	    = .65;    	//Right Open  	1
    static final double Servo0Close 	= .45;     	//Right Close   0

    //LEFT GRABBER ARM
    static final double Servo1Close 	= .55;    	//Left Close   	1
    static final double Servo1Open 	    = .35;     	//Left Open     0

    //FULL OPEN GRABBER ARM
    static final double Servo0FullOpen 	= .95;  	//Right Full Open
    static final double Servo1FullOpen 	= .05;  	//Left Full Open

    //FULL CLOSE GRABBER ARM
    static final double Servo0FullClose = .3; 		//Right super close
    static final double Servo1FullClose = .85;  	//Left super close


    //START AND END PARMS
    double StartTime    = 0.0;
    double EndTime      = 0.0;

    //BELOW VALUES IN SECONDS
        //double CloseArms            = 1;      	//Step 1, CLOSE ARMS AND DROP COLOR ARM
    double DropColorArm         = 1.0;
    double ReadColor            = 1.0;      	//

    double GlyphMotorPowerUp 	=  .6;      	//RAISE GLYPH ARM POWER  WAS .5
    double GlyphMotorPowerDown	= -.5;	    	//LOWER GLYPH ARM POWER, BUT NOT COMPLETELY

    double KnockOffBall         = 1.5;     	//MOVE COLOR ARM LEFT RIGHT

    double ResetMove            = 1.0;      	//REMOVED WHOLE STEP

    double Stop                 = 0.15;     //was .5
    double RaiseColorArm        = 1.0;      	//RAISE COLOR ARM AND RAISE GLYPH ARM

    double StopEncoders         = 0.15;     //was .5
    //
    //
    //
    // 43 Ticks with the Mecanum Chassis
    // 41 Ticks equals about 1 inch
    // 38.5 Ticks NOW Equals about 1 inch with 4 drive motors
    //
    //
    //
    double MoveOffBoard         = 3.0;        	//
    double MOBPower 		    =  .3;		//WAS .3

    //MECANUM CHASSIS
    int MOBClicksLeftLEFT      =  1900;     //LEFT was 1797	Worked 3/31 1941
    int MOBClicksRightLEFT     = -1900;     //LEFT was 1797
    int MOBClicksLeftCENTER    =  1546;     //CENTER      	was 1566
    int MOBClicksRightCENTER   = -1546;     //CENTER
    int MOBClicksLeftRIGHT     =  1153;     //RIGHT		Worked 3/31 1175 1194
    int MOBClicksRightRIGHT    = -1153;     //RIGHT


    //STATE VALUES ORIG CHASSIS
    //int MOBClicksLeftLEFT      = -1838;     //LEFT
    //int MOBClicksRightLEFT     =  1838;     //LEFT
    //int MOBClicksLeftCENTER    = -1525;     //CENTER
    //int MOBClicksRightCENTER   =  1525;     //CENTER
    //int MOBClicksLeftRIGHT     = -1174;     //RIGHT
    //int MOBClicksRightRIGHT    =  1174;     //RIGHT


    double Spin                 = 2.5;
    int Spin90left              = 990;		//Spin Right was 939, 950, 970, 980
    int Spin90right             = 990;
    double SpinPower 		    = .7;
    double SpinPowerFast	    = .9;

    double MoveToWall           = 2.0;
    int MoveToWallClicksLeft    =  500;       	//was 415
    int MoveToWallClicksRight   = -500;
    double MoveToWallPower 	    = .3;

    double DropGlyph            = 1.0;

    double PushGlyph            = 0.0;		//REMOVED TEST TEST TEST
    int PushGlyphClicksLeft     =  492;        	//Forward
    int PushGlyphClicksRight    = -492;
    double PushPower 		    = .4;

    double BackUpFinal          = 1.5;
    int BackUpFinalClicksLeft   = -230;      	//Backward
    int BackUpFinalClicksRight  =  230;
    double BackUpFinalPower 	= .3;


    double encoderaverageleft;
    double encoderaverageright;

    //
    // SERVO CODE
    //
    // RAISE COLOR ARM
    static final double MAX_POS     	=  0;

    // LOWER COLOR ARM
    static final double MIN_POS     	=  0.61; // was .59

    //double  positionx = (MAX_POS - MIN_POS) / 2;

    //Color Arm Sideways
    static final double Servo6Left 	    = .68;  //was .62
    static final double Servo6Right 	= .32;  //was .32
    static final double Servo6Center 	= .5;   //CENTER


    //START PICTO INIT
    //
    public static final String TAG = "Vuforia VuMark Sample";
    OpenGLMatrix lastLocation = null;
    VuforiaLocalizer vuforia;

    String machspeedpicto = "NONE";
    //END PICTO INIT


    @Override
    public void runOpMode() {


        frontleftmotor = hardwareMap.dcMotor.get("frontleftdrive");
        backleftmotor = hardwareMap.dcMotor.get("backleftdrive");
        frontrightmotor = hardwareMap.dcMotor.get("frontrightdrive");
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
        //
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
        //
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
            telemetry.addData("START AUTO RED FRONT: ", 0);

            //telemetry.addData("Motor Output ", "Front Left" + frontleftmotor.getPower());
            //telemetry.addData("Motor Output ", "Front Right" + frontrightmotor.getPower());
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

                telemetry.addData("In VuMark: ", "%s visible", vuMark);
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

                telemetry.addData("SAVED PICTO: ", machspeedpicto);

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
            //}
            //END PICTO
            //


            //
            //START, GRAB GLYPH IN FRONT, CENTER COLOR ARM LEFT RIGHT, DROP COLOR ARM
            //
            StartTime = 0;
            EndTime = StartTime + DropColorArm;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Start, Reset Color Arm, Grab Glyph: ", "1");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                //SET GLYPH ROTATE TO ZERO
                rotateglyph.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //GRAB GLYPH //
                servo0.setPosition(Servo0Close);
                servo1.setPosition(Servo1Close);  	//Left

                //CENTER COLOR LEFT RIGHT MOVEMENT
                servo6.setPosition(Servo6Center);

                //DROP COLOR ARM
                servo2.setPosition(MIN_POS);		//DROP COLOR ARM

                telemetry.addData("DROP ARM: ", MIN_POS);
                telemetry.update();
            }
            //
            //READ COLOR
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + ReadColor;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Read Color, Stop Glyph Arm: ", "2");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                //liftmotor.setPower(0);  	//STOP GLYPH ARM

                // ASSUME WE ARE RED TEAM
                // ASSUME SENSOR FACING BACKWARDS ON ROBOT ON RIGHT SIDE
                // CHECK IF STATEMENT FOR BLUE ONLY
                if (hsvValues[0] >= 130 && hsvValues[0] <= 250) {   //Thursday reading 144 for blue
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
            // KNOCK OFF JEWEL
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
            telemetry.update();
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
            //STOP BEFORE ENCODERS, STOP GLYPH ARM
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Stop;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "5");
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
            //FRONT RED, MOVE OFF BOARD BASED ON VUMARK
            //
            //
            //
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + MoveOffBoard;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Move Off Board: ", "6");
                telemetry.addData("FrontLeftMotor : ", frontleftmotor.getCurrentPosition());
                telemetry.addData("FrontRightMotor: ", frontrightmotor.getCurrentPosition());
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());
                telemetry.update();

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(MOBPower);
                backleftmotor.setPower(MOBPower);
                frontrightmotor.setPower(MOBPower);
                backrightmotor.setPower(MOBPower);

                encoderaverageleft = ((frontleftmotor.getCurrentPosition() +
                        backleftmotor.getCurrentPosition()) / 2);

                encoderaverageright = ((frontrightmotor.getCurrentPosition() +
                        backrightmotor.getCurrentPosition()) / 2);

                telemetry.addData("ENCODER AVERAGE LEFT:  ", encoderaverageleft);
                telemetry.addData("ENCODER AVERAGE RIGHT: ", encoderaverageright);
                telemetry.update();


                if (machspeedpicto == "LEFT") {
                    frontleftmotor.setTargetPosition(MOBClicksLeftLEFT);
                    backleftmotor.setTargetPosition(MOBClicksLeftLEFT);
                    frontrightmotor.setTargetPosition(MOBClicksRightLEFT);
                    backrightmotor.setTargetPosition(MOBClicksRightLEFT);
                    telemetry.addData("LEFT PICTO: ", backleftmotor.getCurrentPosition());
                }
                else if (machspeedpicto == "CENTER") {
                    frontleftmotor.setTargetPosition(MOBClicksLeftCENTER);
                    backleftmotor.setTargetPosition(MOBClicksLeftCENTER);
                    frontrightmotor.setTargetPosition(MOBClicksRightCENTER);
                    backrightmotor.setTargetPosition(MOBClicksRightCENTER);
                    telemetry.addData("CENTER PICTO: ", backleftmotor.getCurrentPosition());
                }
                else if (machspeedpicto == "RIGHT") {
                    frontleftmotor.setTargetPosition(MOBClicksLeftRIGHT);
                    backleftmotor.setTargetPosition(MOBClicksLeftRIGHT);
                    frontrightmotor.setTargetPosition(MOBClicksRightRIGHT);
                    backrightmotor.setTargetPosition(MOBClicksRightRIGHT);
                    telemetry.addData("RIGHT PICTO: ", backleftmotor.getCurrentPosition());
                }
                else {
                    frontleftmotor.setTargetPosition(MOBClicksLeftCENTER);
                    backleftmotor.setTargetPosition(MOBClicksLeftCENTER);
                    frontrightmotor.setTargetPosition(MOBClicksRightCENTER);
                    backrightmotor.setTargetPosition(MOBClicksRightCENTER);
                    telemetry.addData("PICTO FAIL, DEFAULT CENTER: ", backleftmotor.getCurrentPosition());
                }

                //telemetry.addData("MOVE OFF Left Ticks:  ", frontleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE OFF Right Ticks: ", frontrightmotor.getCurrentPosition());
                //telemetry.addData("MOVE OFF Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE OFF Right Ticks: ", backrightmotor.getCurrentPosition());

                telemetry.update();
            }
            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "7");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //
            //FRONT RED, SPIN 90 DEGREES AFTER MOVING OFF THE BOARD
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Spin;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Spin: ", "8");
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

                frontleftmotor.setTargetPosition(Spin90left);
                backleftmotor.setTargetPosition(Spin90left);
                frontrightmotor.setTargetPosition(Spin90right);
                backrightmotor.setTargetPosition(Spin90right);

                //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
            }
            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "9");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //
            //MOVE TO WALL, LOWER GLYPH ARM
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + MoveToWall;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Move to Wall, Lower Glyph Arm: ", "10");
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


                liftmotor.setPower(GlyphMotorPowerDown);		//LOWER GLYPH ARM


                //telemetry.addData("MOVE TO Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE TO Right Ticks: ", backrightmotor.getCurrentPosition());
            }
            //
            //STOP RESET, STOP GLYPH ARM
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "11");

                liftmotor.setPower(0);  	//STOP GLYPH ARM

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //
            //DROP GLYPH, NO MOTOR MOVEMENT
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + DropGlyph;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Drop Glyph: ", "12");

                servo0.setPosition(Servo0Open); 	//DROP GLYPH
                servo1.setPosition(Servo1Open);
            }
            //
            //
            //PUSH GLYPH
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + PushGlyph;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Push Glyph: ", "13");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                servo0.setPosition(Servo0FullOpen);  //Right
                servo1.setPosition(Servo1FullOpen);  //Left

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(PushPower);
                backleftmotor.setPower(PushPower);
                frontrightmotor.setPower(PushPower);
                backrightmotor.setPower(PushPower);

                frontleftmotor.setTargetPosition(PushGlyphClicksLeft);
                backleftmotor.setTargetPosition(PushGlyphClicksLeft);
                frontrightmotor.setTargetPosition(PushGlyphClicksRight);
                backrightmotor.setTargetPosition(PushGlyphClicksRight);
            }
            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "14");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //
            //BACK UP AFTER PUSH
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + BackUpFinal;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Backup Final: ", "15");
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
            //
            //
            //
            if (SPINTOPILE == "YES") {
                //
                //
                //
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
                //
                //
                //SPIN 90 DEGREES TO POINT TO PILE
                //
                //
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Spin 1 To Pile: ", "17");
                    telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                    telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                    frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    frontleftmotor.setPower(SpinPowerFast);
                    backleftmotor.setPower(SpinPowerFast);
                    frontrightmotor.setPower(SpinPowerFast);
                    backrightmotor.setPower(SpinPowerFast);

                    frontleftmotor.setTargetPosition(Spin90left);
                    backleftmotor.setTargetPosition(Spin90left);
                    frontrightmotor.setTargetPosition(Spin90right);
                    backrightmotor.setTargetPosition(Spin90right);

                    //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                    //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
                }

                //
                //STOP AND RESET
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
                //
                //SPIN 90 DEGREES AGAIN FOR 180 TO POINT TO PILE
                //
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Spin 2 To Pile: ", "19");
                    telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                    telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                    frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    frontleftmotor.setPower(SpinPowerFast);
                    backleftmotor.setPower(SpinPowerFast);
                    frontrightmotor.setPower(SpinPowerFast);
                    backrightmotor.setPower(SpinPowerFast);

                    frontleftmotor.setTargetPosition(Spin90left);
                    backleftmotor.setTargetPosition(Spin90left);
                    frontrightmotor.setTargetPosition(Spin90right);
                    backrightmotor.setTargetPosition(Spin90right);


                    //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                    //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
                }
                //
                //STOP AND RESET
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + StopEncoders;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Stop and Reset: ", "20");

                    frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
            }
            //
            //
            //
            //
            //
            //
            //
            //GO FOR SECOND GLYPH
            //
            //
            //
            //

            if (GOFORSECONDGLYPH == "YES") {

                //
                //FORWARD TO PILE
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Move to Pile Fast: ", "25");

                    frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    frontleftmotor.setPower(0.8);
                    backleftmotor.setPower(0.8);
                    frontrightmotor.setPower(0.8);
                    backrightmotor.setPower(0.8);

                    frontleftmotor.setTargetPosition(1404);	 //rougly 36 inches
                    backleftmotor.setTargetPosition(1404);	 //rougly 36 inches
                    frontrightmotor.setTargetPosition(-1404);
                    backrightmotor.setTargetPosition(-1404);

                    //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                    //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
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
                //
                //FORWARD TO PILE SLOW AND GRAB
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Move to Pile Slow and Grab: ", "27");

                    frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    frontleftmotor.setPower(0.3);
                    backleftmotor.setPower(0.3);
                    frontrightmotor.setPower(0.3);
                    backrightmotor.setPower(0.3);

                    frontleftmotor.setTargetPosition(78);	//rougly 2 inches
                    backleftmotor.setTargetPosition(78);	//rougly 2 inches
                    frontrightmotor.setTargetPosition(-78);
                    backrightmotor.setTargetPosition(-78);

                    servo0.setPosition(Servo0Close);  //Right
                    servo1.setPosition(Servo1Close);  //Left

                    //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                    //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
                }
                //
                //STOP AND RESET
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + StopEncoders;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Stop and Reset: ", "28");

                    frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                //
                //SPIN AGAIN 180 WHILE RAISING ARM
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + 4;    //MORE TIME FOR LONGER SPIN

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Spin 180: ", "29");

                    frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                    frontleftmotor.setPower(0.6);
                    backleftmotor.setPower(0.6);
                    frontrightmotor.setPower(0.6);
                    backrightmotor.setPower(0.6);

                    liftmotor.setPower(.8);    //RAISE

                    frontleftmotor.setTargetPosition(1178);
                    backleftmotor.setTargetPosition(1178);
                    frontrightmotor.setTargetPosition(1178);
                    backrightmotor.setTargetPosition(1178);

                    telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                    telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
                }
                //
                //STOP AND RESET, STOP ARM
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + StopEncoders;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Stop and Reset: ", "30");

                    liftmotor.setPower(0);    //STOP

                    frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                //
                //FORWARD TO WALL
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Move to Wall Long: ", "31");

                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setPower(0.6);
                    backrightmotor.setPower(0.6);

                    backleftmotor.setTargetPosition(1150);   //roughly 29 inches
                    backrightmotor.setTargetPosition(-1150);

                    //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                    //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
                }
                //
                //STOP AND RESET
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + StopEncoders;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Stop and Reset: ", "32");

                    //liftmotor.setPower(0);    //STOP GLYPH ARM

                    frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                //
                //FORWARD TO WALL
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Move to Wall Short: ", "33");

                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setPower(0.3);
                    backrightmotor.setPower(0.3);

                    backleftmotor.setTargetPosition(150);
                    backrightmotor.setTargetPosition(-150);

                    //telemetry.addData("SPIN Left Ticks:  ", backleftmotor.getCurrentPosition());
                    //telemetry.addData("SPIN Right Ticks: ", backrightmotor.getCurrentPosition());
                }

                //
                //STOP AND RESET, LOIWER ARM, OPEN ARMS
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + StopEncoders;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Stop and Reset: ", "34");

                    liftmotor.setPower(-.2);    //LOWER

                    servo0.setPosition(Servo0Open);  //Right
                    servo1.setPosition(Servo1Open);  //Left

                    frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }



            }   //END IF GOFORSECONDGLYPH IF
            //
            //
            //
            //
            //STOP AND RESET FINAL, RESET GLYPH ROTOATE BACK TO ZERO
            //
            //
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Final Stop ", "99");

                //rotateglyph.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                //rotateglyph.setPower(.3);
                //rotateglyph.setTargetPosition(0);

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
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
    //
    String format(OpenGLMatrix transformationMatrix) {
        return (transformationMatrix != null) ? transformationMatrix.formatAsTransform() : "null";
    }
    //END PICTO "FORMAT MATRIX"

}
