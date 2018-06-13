/**
 * Created by Mach Speed Programming on 10/31/2017.
 */

// AUTO BLUE FRONT   AUTONOMOUS BY TIME WITH COLOR, DISTANCE and TOUCH CODE
//
// CHANGES
//
// 24-MAR-2018 - Modified Code for Mecanum Wheel Effect
// 02-FEB-2018 - Made sure to shut off Lift Motor correctly
//               Adjusted times to match other Autos
// 15-JAN-2018 - Added Picto Code
//		Added Reset speed to compensate for over rotation
//		Change STOP to 1.0 to speed up Auto
// 11-JAN-2018 - Added Full Close for Servo Grabber Arms
//               Corrected servo Open and Close values
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


@Autonomous(name = "AutoBlueFront", group = "Sensor")
//@Disabled                            // Comment this out to add to the opmode list

public class AutoBlueFront extends LinearOpMode {

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
    Servo servo6;  //Sensor Arm Sideways

    DigitalChannel digitaltouch;  // Hardware Device Object

    ColorSensor sensorcolor;
    DistanceSensor sensordistance;

    Boolean Isblue;


    String SPINTOPILE 		= "NO";  //YES = SPIN ROBOT TO FACE PILE OF GLYPHS, NO = POINT AT CRYPTOBOX


    int ColorBlue 	= 0;
    int pictocount 	= 1;         //this is for PICTO

    //RIGHT GRABBER ARM
    static final double Servo0Open 	    = .65;    //Right Open        1 = open
    static final double Servo0Close 	= .45;     //Right Close     0 = close

    //LEFT GRABBER ARM
    static final double Servo1Close 	= .55;    //Left Close       1 = close
    static final double Servo1Open    	= .35;     //Left Open        0 = open

    //FULL OPEN GRABBER ARM
    static final double Servo0FullOpen 	= .95;        //Right Full Open
    static final double Servo1FullOpen 	= .05;    //Left Full Open

    //FULL CLOSE GRABBER ARM
    static final double Servo0FullClose = .3;        //Right super close
    static final double Servo1FullClose = .85;    //Left super close


    //START AND END PARMS
    double StartTime    = 0.0;
    double EndTime      = 0.0;

    //BELOW VALUES IN SECONDS
            //double CloseArms 		        = 1.0;      	//Step 1, CLOSE ARMS AND DROP COLOR ARM
    double DropColorArm             = 1.0;
    double ReadColor 		        = 1.0;      	//

    double GlyphMotorPowerUp 	    =  .5;          //RAISE GLYPH ARM POWER
    double GlyphMotorPowerDown 	    = -.4;        //LOWER GLYPH ARM POWER, BUT NOT COMPLETELY

    double KnockOffBall 	        = 1.5;        	//

    double ResetMove 		        = 1.0;          //

    double Stop 	 	            = 0.15;      	//was 2.0
    double RaiseColorArm 	        = 1.0;      	//

    double StopEncoders 	        = 0.15;      	//
    //
    //
    //
    // 43 Ticks with the Mecanum Chassis
    // 41 Ticks equals about 1 inch
    // 38.5 Ticks NOW Equals about 1 inch with 4 drive motors
    //
    //
    //
    double MoveOffBoard 		     = 3;        // was 2

    //MECANUM CHASSIS
    int MTGClicksLeftLEFT 	    	= -1283;     //LEFT - Forward Right Spin
    int MTGClicksRightLEFT 	    	=  1283;     //LEFT - Forward Right Spin
    int MTGClicksLeftCENTER 		= -1600;     //CENTER - Forward Right Spin
    int MTGClicksRightCENTER 		=  1600;     //CENTER - Forward Right Spin
    int MTGClicksLeftRIGHT 	    	= -1947;     //RIGHT - Forward Right Spin
    int MTGClicksRightRIGHT 		=  1947;     //RIGHT - Forward Right Spin

    double MOBPower 		    	= .3;	//was .3

    //STATE VALUES ORIG CHASSIS
    //int MTGClicksLeftLEFT 	=  1119;     //LEFT - Forward Right Spin       was -1877 1213 1194 1156 1100
    //int MTGClicksRightLEFT 	= -1119;     //LEFT - Forward Right Spin        was 1722  1194 1156 1100
    //int MTGClicksLeftCENTER 	=  1429;     //CENTER - Forward Right Spin         was 1525 1602 1448 1429
    //int MTGClicksRightCENTER 	= -1429;     //CENTER - Forward Right Spin       was -731 1525 1602 1448 1429
    //int MTGClicksLeftRIGHT 	=  1763;     //RIGHT - Forward Right Spin  was 1098 1916 1877 1725
    //int MTGClicksRightRIGHT 	= -1763;     //RIGHT - Forward Right Spin   was 1098 1687

    double Spin 			         = 3.0;
    int Spin90left 			         = 970; //
    int Spin90right 			     = 970; //
    double SpinPower 			     = .7;

    double MoveToWall 			    = 2.0;        //
    int MoveToWallClicksLeft 		=  500;     //Forward was 164 231
    int MoveToWallClicksRight 		= -500;      //Forward was 164 231
    double MoveToWallPower 		    = .3;

    double DropGlyph 			     = 1.0;      //

    double PushGlyph 			     = 0;        //REMOVED TEST TEST TEST
    int PushGlyphClicksLeft 		=  569;     //Forward was -492
    int PushGlyphClicksRight 		= -569;      //Forward was 492
    double PushPower 			    = .4;

    double BackUpFinal 			    = 1.5;        //Was .05
    int BackUpFinalClicksLeft 		= -230;      //Backward was 115
    int BackUpFinalClicksRight 		=  230;     //Backward
    double BackUpFinalPower 		= .3;

    //
    // SERVO CODE
    //
    // RAISE COLOR ARM
    static final double MAX_POS 	= 0;

    // LOWER COLOR ARM
    static final double MIN_POS 	= 0.61;

    //double positionx = (MAX_POS - MIN_POS) / 2;

    //Color Arm Sideways
    static final double Servo6Left 	    = .32; 	//was .6, Reversed from Red Side
    static final double Servo6Right 	= .68; 	//was .4, Reversed from Red Side
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
        frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontrightmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backleftmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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

            telemetry.addData("START AUTO BLUE FRONT: ", 0);

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

                telemetry.addData("VuMark Chuck1 ", "%s visible", vuMark);
                //telemetry.addData("Location ", vuMark);
                //machspeedpicto = vuMark;

                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    machspeedpicto = "LEFT";
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    machspeedpicto = "CENTER";
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    machspeedpicto = "RIGHT";
                } else
                    machspeedpicto = "CENTER"; {
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
                //telemetry.addData("Before PictoCount 2 = ", pictocount);
                pictocount = pictocount + 1;
                //telemetry.addData("After PictoCount 2 = ", pictocount);
            } else {
                telemetry.addData("Else VuMark ", "not visible");
                //telemetry.addData("Before PictoCount 3 = ", pictocount);
                pictocount = pictocount + 1;
            }
            //    telemetry.addData("Else PictoCount 4 = ",pictocount);

            //}

            telemetry.update();

            //if (vuMark == RelicRecoveryVuMark.CENTER) {
            //    telemetry.addData("SAVED CENTER ", vuMark);
            //
            //}
            //END PICTO
            //


            //
            //START, GRAB GLYPH IN FRONT, DROP COLOR ARM
            //
            StartTime = 0;
            EndTime = StartTime + DropColorArm;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Start Reset Color Arm, Grab Glpyh: ", "1");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                //SET GLYPH ROTATE TO ZERO
                rotateglyph.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                //GRAB GLYPH //
                servo0.setPosition(Servo0Close);
                servo1.setPosition(Servo1Close);  //Left

                //CENTER COLOR LEFT RIGHT MOVEMENT
                servo6.setPosition(Servo6Center);

                //DROP COLOR ARM
                servo2.setPosition(MIN_POS);		//DROP COLOR ARM
                telemetry.addData("DROP ARM: ", MIN_POS);
                telemetry.update();
            }

            //
            //READ COLOR, STOP GLYPH LIFT
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + ReadColor;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Read Color: ", "3");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                //liftmotor.setPower(0);    //STOP RAISING GLYPH ARM

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
                telemetry.addData("Knock Off Ball: ", "4");
                //telemetry.addData("StartTime   ", StartTime);
                //telemetry.addData("EndTime     ", EndTime);

                if (ColorBlue == 1) {
                    telemetry.addData("Knock Off RED Ball: ", "Running: " + runtime.toString());

                    servo6.setPosition(Servo6Right);
                    telemetry.addData("Servo6Right: ", + Servo6Right);
                } else {
                    telemetry.addData("Knock off BLUE Ball: ", "Running: " + runtime.toString());

                    servo6.setPosition(Servo6Left);
                    telemetry.addData("Servo6Left: ", + Servo6Left);
                }
            }
            //
            //RAISE COLOR ARM. RAISE GLYPH ARM
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + RaiseColorArm;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Raise Color Arm: ", "5");
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
                telemetry.addData("STOP After Reset: ", "6");
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
            //FRONT BLUE, DO MOVE OFF THE BOARD
            //
            //
            //
            //
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + MoveOffBoard;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Move Off Board: ", "7");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                backrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

                frontleftmotor.setPower(MOBPower);
                frontrightmotor.setPower(MOBPower);
                backleftmotor.setPower(MOBPower);
                backrightmotor.setPower(MOBPower);

                if (machspeedpicto == "LEFT") {
                    frontleftmotor.setTargetPosition(MTGClicksLeftLEFT);
                    backleftmotor.setTargetPosition(MTGClicksLeftLEFT);
                    frontrightmotor.setTargetPosition(MTGClicksRightLEFT);
                    backrightmotor.setTargetPosition(MTGClicksRightLEFT);
                    telemetry.addData("LEFT PICTO: ", backleftmotor.getCurrentPosition());
                } else if (machspeedpicto == "CENTER") {
                    frontleftmotor.setTargetPosition(MTGClicksLeftCENTER);
                    backleftmotor.setTargetPosition(MTGClicksLeftCENTER);
                    frontrightmotor.setTargetPosition(MTGClicksRightCENTER);
                    backrightmotor.setTargetPosition(MTGClicksRightCENTER);
                    telemetry.addData("CENTER PICTO: ", backleftmotor.getCurrentPosition());
                } else if (machspeedpicto == "RIGHT") {
                    frontleftmotor.setTargetPosition(MTGClicksLeftRIGHT);
                    backleftmotor.setTargetPosition(MTGClicksLeftRIGHT);
                    frontrightmotor.setTargetPosition(MTGClicksRightRIGHT);
                    backrightmotor.setTargetPosition(MTGClicksRightRIGHT);
                    telemetry.addData("RIGHT PICTO: ", backleftmotor.getCurrentPosition());
                } else {
                    frontleftmotor.setTargetPosition(MTGClicksLeftCENTER);
                    backleftmotor.setTargetPosition(MTGClicksLeftCENTER);
                    frontrightmotor.setTargetPosition(MTGClicksRightCENTER);
                    backrightmotor.setTargetPosition(MTGClicksRightCENTER);
                    telemetry.addData("PICTO FAIL, DEFAULT CENTER: ", backleftmotor.getCurrentPosition());
                }

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
                telemetry.addData("Stop and Reset: ", "8");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //SPIN AFTER MOVE OFF BOARD
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + Spin;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Spin: ", "9");
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
            }
            //
            //STOP AND RESET
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop: ", "10");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //MOVE TO WALL, LOWER GLYPH ARM
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + MoveToWall;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Move to Wall, Lower Glyph Arm: ", "11");
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


                liftmotor.setPower(GlyphMotorPowerDown);    //LOWER GLYPH ARM


                //telemetry.addData("MOVE TO Left Ticks:  ", backleftmotor.getCurrentPosition());
                //telemetry.addData("MOVE TO Right Ticks: ", backrightmotor.getCurrentPosition());
            }
            //
            //STOP RESET, STOP LIFT MOTOR
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Stop and Reset: ", "12");

                liftmotor.setPower(0);    //STOP GLYPH ARM

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
                telemetry.addData("Drop Glyph: ", "13");

                servo0.setPosition(Servo0Open);		//DROP GLYPH
                servo1.setPosition(Servo1Open);
            }
            //
            //PUSH GLYPH
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + PushGlyph;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Push Glyph: ", "14");
                telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

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
                telemetry.addData("Stop and Reset: ", "15");

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
            //
            //BACK UP AFTER PUSH
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + BackUpFinal;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("Encoder, Backup Final: ", "16");
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
                //
                //STOP AND RESET
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + StopEncoders;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Stop and Reset: ", "17");

                    frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }
                //
                //SPIN 90 DEGREES TO POINT TO PILE
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Spin 1 To Pile: ", "18");
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
                    frontrightmotor.setTargetPosition(Spin90right);
                    backleftmotor.setTargetPosition(Spin90left);
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
                    telemetry.addData("Stop and Reset: ", "19");

                    frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                }

                //
                //SPIN 90 DEGREES AGAIN FOR 180 TO POINT TO PILE
                //
                StartTime = EndTime + .1;
                EndTime = StartTime + Spin;

                if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                    telemetry.addData("Spin 2 To Pile: ", "20");
                    telemetry.addData("BackLeftMotor : ", backleftmotor.getCurrentPosition());
                    telemetry.addData("BackRightMotor: ", backrightmotor.getCurrentPosition());

                    frontleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    frontrightmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    backleftmotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
            }
            //
            //STOP AND RESET FINAL
            //
            StartTime = EndTime + .1;
            EndTime = StartTime + StopEncoders;

            if (runtime.seconds() >= StartTime && runtime.seconds() <= EndTime) {
                telemetry.addData("FINAL STOP: ", "21");

                /*rotateglyph.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                rotateglyph.setPower(.3);
                rotateglyph.setTargetPosition(0);*/

                frontleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backleftmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                frontrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                backrightmotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }
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

            telemetry.update();        //SEND TELEMETRY TO PHONE
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


