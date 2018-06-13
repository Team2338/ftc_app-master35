/**
 * Created by Mach Speed Programming on 02/11/2018.
 */
//
//TELEOP TO CENTER SERVOS
//
// 12-FEB-2018 - Modified TeleOp2 to Center All of our Servos for Setup
//
//
package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import java.util.Locale;  //Needed for Color Sensor


@TeleOp(name="Center", group="Iterative OpMode")  // @Autonomous(...) is the other common choice
//@Disabled

public class Center extends OpMode {
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


    Servo servo0;	//Hub1 Port 0, Grabber Bottom Right
    Servo servo1;	//Hub1 Port 1, Grabber Bottom Left

    Servo servo2;	//Hub1 Port 2, Sensor Arm
    Servo servo3;	//Hub1 Port 3, Relic Arm

    Servo servo4;	//Hub1 Port4, Grabber Top
    Servo servo5;	//Hub1 Port5, Grabber Top

    Servo servo6;	//Color Arm Sideways




    DigitalChannel digitaltouch;  // Hardware Device Object

    ColorSensor sensorcolor;
    DistanceSensor sensordistance;


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {

        telemetry.addData("Status", "Initialized Setting Servos");

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


        servo0 = hardwareMap.servo.get("servo0"); //Bottom Right Grabber
        servo1 = hardwareMap.servo.get("servo1"); //Bottom Left Grabber
        servo2 = hardwareMap.servo.get("servo2"); //Color Arm
        servo3 = hardwareMap.servo.get("servo3"); //Relic Grabber
        servo4 = hardwareMap.servo.get("servo4"); //Top Right Grabber
        servo5 = hardwareMap.servo.get("servo5"); //Top Left Grabber
        servo6 = hardwareMap.servo.get("servo6"); //Top Left Grabber


    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        telemetry.addData("Status", "Running, Setting Servos to 90 Degrees: " + runtime.toString());


        servo0.setPosition(.5);   	//Bottom Right Grabber 90 degrees
        servo1.setPosition(.5);  	//Bottom Left Grabber 90 degrees
        servo2.setPosition(.5);   	//Color Arm 90
        servo3.setPosition(.5);   	//Relic Arm 90
        servo4.setPosition(.5);   	//Top Right Grabber 90
        servo5.setPosition(.5);   	//Top Right Grabber 90
        servo6.setPosition(.5);   	//Color Arm Sideways
        //servo6.setPosition(.5);


/*
        //
        //IF STATEMENT EXAMPLE
        //
        if (gamepad2.left_bumper) {    
            telemetry.addData("IN ROTATE LEFT: ", rotate.getCurrentPosition());
            servo1.setPosition(.5);        //Left Open
            servo0.setPosition(.5);        //Right Open
        }
        else if (gamepad2.right_bumper) {
            telemetry.addData("IN ROTATE RIGHT: ", rotate.getCurrentPosition());
        }
            else{
               rotate.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE); // ADDED BY CHUCK Feb 11
               rotate.setPower(0);
            }
*/

    }

    @Override
    public void stop() {
    }
}       