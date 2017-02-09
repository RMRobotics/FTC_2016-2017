package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;


import android.graphics.Bitmap;
import android.widget.ImageView;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;

import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.rmrobotics.R;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.ArrayBlockingQueue;

import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import pm.BeaconRecognizer;
import pm.ButtonFinder;


/**
 * Created by Peter on 12/15/2016.
 */

@Autonomous(name = "BetterDarudeAutoNav", group = "AutoNav")
public class BetterDarudeAutoNav extends LinearOpMode {

    //runtime calculations
    static ElapsedTime runtime = new ElapsedTime();

    RMVuforia vuforia;

    //motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    private Drive2 drive = null;

    //servo
    CRServo leftPusher;
    CRServo rightPusher;
    Servo latch;

    //navx
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    I2cDeviceSynch rangeSensor = null;

    private ImageView mImageView;

    private double dir = 1;

    private final int RECOGNITION_DIST_Y = 330; // Distance to turn on OpenCV
    private final int RECOGNITION_TOLER_Y = 40; // Positioning tolerance along Y
    private final int RECOGNITION_TOLER_X = 30; // Positioning tolerance along Y
    private final int REPOSITION_DIST = 480; // Distance to back up if out of position to recognize button

/*
    @Override
    public void runOpMode() throws InterruptedException {
        BeaconTracker bt = new BeaconTracker(
                (CameraBridgeViewBase) ((FtcRobotControllerActivity) hardwareMap.appContext).findViewById(R.id.camera_view),
                this,
                true);
        ADBLog("Created OpenCV");
        //first runthru variable
        boolean start = true;
        int verify = 0;
        while (opModeIsActive()) {
            if (bt.GetState() == Tracker.State.RECOGNIZING) {
                //trying to find button
                sleep(30);
                ADBLog("Recognizing");
            } else if (bt.GetState() == Tracker.State.RECOGNIZED) {
                ADBLog("Recognized. Starting approach.");
                bt.SetState(Tracker.State.TRACKING);
                drive.resetDistance();
            } else if (bt.GetState() == Tracker.State.LOST) {
                // Lost button.  FeelsBadMan Todo: Try to recover button somehow !!
                sleep(5);
                ADBLog("Lost button.");
                continue;
            } else if (bt.GetState() == Tracker.State.TRACKING) {
                RotatedRect btn = bt.getBtnPosition();
                if (btn == null) {
                    sleep(5); // No updates yet, wait
                    continue;
                }
            }
        }
    }
    */

    @Override
    public void runOpMode() throws InterruptedException {
        {

            // Read config file
            AutoNavConfig cfg = new AutoNavConfig();
            cfg.ReadConfig(((FtcRobotControllerActivity) hardwareMap.appContext));
            if (cfg.isRight) dir = 1;
            else dir = -1;

            //init vuforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(/*com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId*/);
            parameters.vuforiaLicenseKey = "AfMO3Zb/////AAAAGRP6yrVXWkBalSEp+NZax44NTzFOkclxZo99uF0DhcZXRp0O2qHl6wcPx8Bp+3dppnmePU1HwDeEwSmWl5k/QEHiTAXMEfc/DBjkAVcMvEBnJGaGEHjKsD4/YaU+cdVcU7Q+NbG7fT1KYcpRUu2btnjhHWCPPhS0mu2AtWNw87FuXX0ob8GCP9jc7fKqVkdgChkshd9aaqJ6113IosFisru2Jk2V098iWv20c3ASuEp0oZfFp6DtLKjx0GQWc7+PhM8rxNRRFNbvwTKAUeiQBRyDoh+UroSO39fon2BcFrStL6pEKa6pqIahZGaeIZXcbx9KpmfO6LNX2vkIgdbw+KZjcszjI0x1RYZDcCu4U5V6";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            parameters.useExtendedTracking = false;

            vuforia = new RMVuforia(parameters);
            Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
            VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

            // Enable frame grabbing
            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true); //This line is very important, make sure the keep the format constant throughout the program. I'm using the MotoG2. I've also tested on the ZTE speeds and I found that they use RGB888
            vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

            mImageView = (ImageView) ((FtcRobotControllerActivity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.imageViewId);

            //gets trackable objects and puts them in an array
            VuforiaTrackable wheels = targets.get(0);
            wheels.setName("Wheels");
            VuforiaTrackable tools = targets.get(1);
            tools.setName("Tools");
            VuforiaTrackable legos = targets.get(2);
            legos.setName("Legos");
            VuforiaTrackable gears = targets.get(3);
            gears.setName("Gears");

            VuforiaTrackable firstTarget = null;
            VuforiaTrackable secondTarget = null;

            //Sets two targets to be the ones that will be tracked based on config.

            for (int it = 0; it < 4; it++) {
                if (targets.get(it).getName().equals(cfg.firstBeacon))
                    firstTarget = targets.get(it);
                if (targets.get(it).getName().equals(cfg.secondBeacon))
                    secondTarget = targets.get(it);
            }
            targets.activate();

//            List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
//            allTrackables.addAll(targets);

//            // Enable frame grabbing
//            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true); //This line is very important, make sure the keep the format constant throughout the program. I'm using the MotoG2. I've also tested on the ZTE speeds and I found that they use RGB888
//            vuforia.setFrameQueueCapacity(1); //tells VuforiaLocalizer to only store one frame at a time

            //init motors
            frontLeft = hardwareMap.dcMotor.get("wheelFL");
            frontRight = hardwareMap.dcMotor.get("wheelFR");
            backLeft = hardwareMap.dcMotor.get("wheelBL");
            backRight = hardwareMap.dcMotor.get("wheelBR");

            //and servo
            leftPusher = hardwareMap.crservo.get("leftP");
            leftPusher.setPower(0);
            rightPusher = hardwareMap.crservo.get("rightP");
            rightPusher.setPower(0);

            latch = hardwareMap.servo.get("latch");
            latch.setPosition(0.5);

            // Create NAVX device

            DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get("dim");
            ;
            ADBLog(dim.toString());

            navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                    NAVX_DIM_I2C_PORT,
                    AHRS.DeviceDataType.kProcessedData,
                    NAVX_DEVICE_UPDATE_RATE_HZ);

            ADBLog("NavX is ready");

            // Create optical sensor
            rangeSensor = new I2cDeviceSynchImpl(
                    hardwareMap.i2cDevice.get("range"),
                    I2cAddr.create8bit(0x62), // Sensor I2C address.
                    false);
            rangeSensor.engage();
            //drive class for big roboto
            drive = new Drive2(
                    frontLeft,
                    frontRight,
                    backLeft,
                    backRight,
                    navx_device,
                    telemetry,
                    this
            );
            drive.ReverseDirection();

            // To test drive train set TestDriveTrain to true
            boolean TestDriveTrain = false;
            if (TestDriveTrain) {

                drive.testFrontLeft(1);
                sleep(1000);
                drive.testFrontLeft(0.0);
                sleep(1000);

                drive.testFrontRight(1);
                sleep(1000);
                drive.testFrontRight(0.0);
                sleep(1000);

                drive.testBackLeft(1);
                sleep(1000);
                drive.testBackLeft(0.0);
                sleep(1000);

                drive.testBackRight(1);
                sleep(1000);
                drive.testBackRight(0.0);
                sleep(1000);


                stop();
                if (true) return;
            }


            // Initialize display view
//            mImageView = (ImageView) ((FtcRobotControllerActivity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.imageViewId);


            targets.activate();
            float x = 0;
            float y = 0;
            Mat img = null;

            sleep(500);
            ADBLog("Initialization complete");
            telemetry.clear();
            telemetry.addData("Finished", "the initialization");
            telemetry.update();
            waitForStart();
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
// Start here
            // Start the engine!!!
            // 1 encoder tick ~ 1 mm

            try {

                ADBLogReset();
                ADBLog("Start");
                new Thread(drive).start();


//                drive.TurnToAngle(90);
//                sleep(2000);
//
//                drive.TurnToAngle(45);
//                sleep(2000);

//                drive.DriveByEncoders(0, 0.3, 100);
//                drive.DriveByEncoders(45, 0.3, 1200);
//                drive.brake();

//
//                drive.resetYDist();
//                drive.VecDriveBalanced(-3,10,0.3,2000);
//                sleep(3000);
//                ADBLog("Strafe: " + drive.getYDistIncr());
//
//                if (true) {
//                    return;
//                }


                drive.DriveByEncoders(0, 0.3, 100);
                ADBLog("Step 1");
                drive.DriveByEncoders(35 * dir, 0.4, 500);
                drive.DriveByEncoders(35 * dir, 0.5, 200);
                drive.DriveByEncoders(35 * dir, 0.5, 100);
                ADBLog("Step 2");
                drive.DriveByEncoders(0 * dir, 0.5, 300);
                drive.DriveByEncoders(0 * dir, 0.5, 2);
                drive.DriveByEncoders(0 * dir, 0.4, 200);
                drive.brake();
                ADBLog("Step 4");
                drive.TurnToAngle(90 * dir);
                ADBLog("Step 5");



                // Approaching first target
                double d = 1000;
                while (opModeIsActive()) {
                    ADBLog("Starting approach");
                    d = Approach(firstTarget);
                    ADBLog("Approached with Xerror = " + d);

                    if (Math.abs(d) < RECOGNITION_TOLER_X) {
                        boolean success = RecognizeAndPush(cfg.isRed);
                        if (success) break;
                    }
                    // Try to reposition
                    ADBLog("Repositioning");
                    int back = REPOSITION_DIST - RangeDist();

                    drive.DriveByEncoders(90 * dir, -0.2, back);
                    drive.brake();
                }


///////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////
                // Go to the second beacon
                ADBLog("Step 1-2");
                drive.DriveByEncoders(90 * dir, -0.3, 100);
                ADBLog("Step 2-2");
                drive.DriveByEncoders(172 * dir, -0.4, 400);
                drive.DriveByEncoders(172 * dir, -0.5, 700);
                drive.DriveByEncoders(172 * dir, -0.5, 900);
                drive.brake();
                ADBLog("Step 3-2");
                drive.TurnToAngle(90 * dir);
                ADBLog("Step 5");

                rightPusher.setPower(0);
                leftPusher.setPower(0);

                // Approaching first target
                while (opModeIsActive()) {
                    ADBLog("Starting approach");
                    d = Approach(secondTarget);
                    ADBLog("Approached with Xerror = " + d);

                    if (Math.abs(d) < RECOGNITION_TOLER_X) {
                        boolean success = RecognizeAndPush(cfg.isRed);
                        if (success) break;
                    }
                    // Try to reposition
                    ADBLog("Repositioning");
                    int back = REPOSITION_DIST - RangeDist();

                    drive.DriveByEncoders(90 * dir, -0.2, back);
                    drive.brake();
                }

                rightPusher.setPower(0);
                leftPusher.setPower(0);

                drive.DriveByEncoders(90 * dir, -0.2, 100);
                drive.brake();
                sleep(50);

            } finally {
                ADBLog("Exiting opmode");
                drive.Stop();
                leftPusher.setPower(0);
                leftPusher.close();
                rightPusher.setPower(0);
                rightPusher.close();
                latch.setPosition(0.5);
                latch.close();
                navx_device.close();
                stop();
            }
        }
    }

    private volatile static double initial_time = 0;

    static synchronized public void ADBLogReset() {
        initial_time = runtime.milliseconds();
        RobotLog.d("Initial time: " + initial_time);
    }

    static synchronized public void ADBLog(String msg) {
        double t = -initial_time;
        t += runtime.milliseconds();
        RobotLog.d(String.format("%.0f", t) + ": " + msg);
    }

    private int RangeDist() {
        byte[] array = rangeSensor.read(0x04, 2);
        //amplifies the y value to emphasize it in the vector (it is in centimeters so it needs to be increased)
        return array[0] * 10;
    }

    static final double Y_DIST = 150;
    static final double X_TOLERANCE = 20;
    Integrator I = new Integrator(400);
    double prevX = 0;
    double prevY = 0;
    double prevT = runtime.milliseconds();

    enum States {OFF_LEFT, OFF_RIGHT, ON_TARGET}

    ;

    private double Approach(VuforiaTrackable target) {
        double x = 0;
        double y = 1000;
        double offset = 0;
        int angle = 0;
        int prevAngle = 0;
        double d = 1000;
        double head = 90 * dir;

        // First intercept perpendicular line
/*        while (opModeIsActive())
        {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getRawUpdatedPose();
            if(((VuforiaTrackableDefaultListener) target.getListener()).isVisible()) {
                if (pose != null) {
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 16);
                    x = poseData[3];
                    y = poseData[11];

                    double b = Math.atan(x / y);
                    double a = Math.toRadians(navx_device.getYaw()-head);

                    double r = Math.sqrt(x * x + y * y);
                    x = r * Math.sin(b + a);
                    offset = 170 * Math.sin(a);
                    ADBLog("Angles: a= " + a / Math.PI * 180 + ", b= " + b / Math.PI * 180);
                    ADBLog("Coords: x'= " + x + ", y'= " + y + ", r=" + r + ", x= " + x + ", offset=" + offset);

                    d = (x + offset)*1;

                    if(Math.abs(d) > 70) {

                        int dist = (int) Math.abs(Math.sqrt(2) * d);

                        ADBLog("Driving for: " + dist);
                        if (d < 0) {
                            drive.TurnToAngle(-30 + head);
                            drive.DriveByEncoders(-30 + head, 0.17, dist);
                            drive.brake();
                        } else {
                            drive.TurnToAngle(30 + head);
                            drive.DriveByEncoders(30 + head, 0.17, dist);
                            drive.brake();
                        }
                    }

                    drive.brake();
                    drive.TurnToAngle(head);
//                    sleep(200);
                    break;
                } else {
                    //continue;
                    ADBLog("Continue");
                }
            } else {
                ADBLog("Lost beacon. What to do?");
            }
            sleep(10);
        }*/


        VectorF vuf_coord = new VectorF(0, 0);
        while (!getVuforiaCoord(target, vuf_coord, 100)) {
            ADBLog("Lost Vurforia track. Do something!!!");
            sleep(30);
        }

        x = prevX = vuf_coord.getData()[0] - 10;
        y = prevY = getRangeSensor();
        drive.resetYDist();
        I.Reset();
        boolean retesting = false;
        ADBLog("Intercepted. Coords: X:" + x + ", Y:" + y);

        // Follow perpendicular
        int retest = 0;
        while (opModeIsActive()) {
            double mult = 1;
            if (((VuforiaTrackableDefaultListener) target.getListener()).isVisible()) {
                if (!getVuforiaCoord(target, vuf_coord, 100)) {
                    // Lost Vuforia track! Do something!
                    ADBLog("Lost Vurforia track. Do something!!!");
                    continue;
                }

                x = (vuf_coord.getData()[0] + 20);
                y = getRangeSensor();

                double currT = runtime.milliseconds();
                // Get incremental data from drive
                double Xd = drive.getYDistIncr();

                I.Add(Xd, currT);

                double xi = 1.5 * I.Get(currT);
                double pX = x - xi;
                double XE = pX;
                double absXE = Math.abs(XE);
                double XV = (prevX - x) / (currT - prevT);

                double YE = y - RECOGNITION_DIST_Y;
                double absYE = Math.abs(YE);
                double YV = (prevY - y) / (currT - prevT);

                double x_dir = 2*Math.signum(XE);
                double y_dir = Math.signum(YE);

                double sp = 0.3;

                // Set speed according to Y distance
                if (absYE < 100) {
                    sp = 0.23;
                }

                if (absYE < 50) {
                    if (Math.abs(YV) < 0.01) {
                        if(!retesting) {
                            retesting = true;
                            sleep(300);
                        } else break; // Got into position
                    }
                    drive.brake(); // Still moving, check later
                    x_dir = 0;
                    y_dir = 0;
                } else {
                    // Set speed according to direction of Y movement
                    if (YV == 0.0 || Math.signum(YV) == Math.signum(YE)) {
                        // Going in right direction
                        if ((absYE < RECOGNITION_TOLER_Y + 30) && (Math.abs(YV) > 0.1)) y_dir = 0;
                    } else {
                        // Going in wrong direction
                        if (Math.abs(YV) > 0.05) y_dir = -y_dir;
                    }

                    // Stop strafing if prediction is past target and Vuforia didn't catch up yet. Don't go back.
                    if (Math.signum(x) != Math.signum(pX)) {
                        x_dir = 0;
                    } else {
                        if (XV == 0.0 || Math.signum(XV) == Math.signum(XE)) { // Check whether going into right direction
                            if (absXE < RECOGNITION_TOLER_X/2) {
                                x_dir = 0; // Within tolerance, stop strafing
                            }
                            if (absXE < RECOGNITION_TOLER_X + 90) {
                                x_dir /= 2; // Close to target and moving fast, stop strafing
                            }
                            if (absXE < RECOGNITION_TOLER_X + 30 && Math.abs(XV) > 0.1) {
                                x_dir = 0; // Close to target and moving fast, stop strafing
                            }
                        } else {
                            //if(Math.abs(XV) > 0.02) x_dir = -x_dir;
                            if (absXE < RECOGNITION_TOLER_X) {
                                x_dir = 0; // Within tolerance, stop strafing.
                            }
                            // Moving in wrong direction and not close to target X. Continue and hope it sorts itself out.
                        }
                    }
                }

                ADBLog("x:" + x + ", xi:" + xi + ", y:" + y + ", pX:" + pX + ", XE:" + XE + ", YE:" + YE + ", XV: " + XV + ", YV:" + YV + ", xd:" + x_dir + ", yd:" + y_dir + ", sp:" + sp);
                drive.VecDriveBalanced(y_dir, x_dir, sp, 2000);


                prevX = x;
                prevY = y;
                prevT = currT;

/*

                    double sp = 0.2;
                    double dist = Math.sqrt(x*x + (y- RECOGNITION_DIST_Y)*(y- RECOGNITION_DIST_Y));

                    int duration = 200;
                    int delay = 10;
                    if(dist < 100) {
                        duration = 150;
                        delay = 100;
                        sp = 0.6;
                        drive.brake();
                    } else if (dist < 200) {
                        sp = 0.1;
                    }

                    ADBLog("Coords: x= " + x + ", y= " + y + ", sp=" + sp + ", duration=" + duration + ", time=" + retest);

                    if (Math.abs(y - RECOGNITION_DIST_Y) < RECOGNITION_TOLER_Y
                        && Math.abs(x) < RECOGNITION_TOLER_X ) {
                        drive.brake_and_wait();
                        ADBLog("In position");
                        if(retest > 0) break;
                        retest = 1;
                        sleep(80);
                        continue;
                    } else retest = 0;

                    //if (Math.abs(y - 420) < 100) sp = 0.2;

                    drive.resetDistance();
                    sleep(delay);
                    drive.VecDriveBalanced(y - RECOGNITION_DIST_Y, x, sp*mult, 500);
                    sleep(duration);
                    if(drive.getDistance() < 5) {
                        mult = 2;
                    }
                    else if(drive.getDistance() > 30) {
                        mult = 1;
                    }
                    ADBLog("Dist: " + drive.getDistance() + ", Mult: " + mult);
                    */
            }
        }

        ADBLog("Brake");
        drive.brake();

        return x;
    }

    private boolean RecognizeAndPush(boolean isRed) {
        ButtonFinder.EllipseLocationResult btn = null;
        BeaconRecognizer br = new BeaconRecognizer();

        double start_time = runtime.milliseconds();

        while(opModeIsActive()) {
            ADBLog("Recognizing");
            Mat img = GetCameraImage();
            ADBLog("Recognizing, got image");
            img = img.t();
            btn = br.detectButtons(img, isRed);
            ADBLog("Recognizing, got button");
            img = img.t();
            DisplayImage(img);
            if (btn != null) {
                break;
            }
            ADBLog("Cannot find button in the image");
            sleep(10);
            if (runtime.milliseconds() - start_time > 1000) {
                // Cannot locate beacon, retry approach
                ADBLog("Cannot locate beacon, retry approach");
                return false;
            }
        }
        if(btn == null) return false;

        /*
        boolean located = false;
        // OpenCV beacon tracking class
        BeaconTracker bt = null;
        boolean isRight = false;
        // here goes code for poking the button via image recognition
        try {
            // Start OpenCV
            // Starting video stream
            bt = new BeaconTracker(
                    (CameraBridgeViewBase) ((FtcRobotControllerActivity) hardwareMap.appContext).findViewById(R.id.camera_view),
                    this,
                    isRed);
            ADBLog("Created OpenCV");

            double start_time = runtime.milliseconds();

            while (opModeIsActive()) {
                if (bt.GetState() == Tracker.State.RECOGNIZING) {
                    //trying to find button
                    sleep(50);
                    ADBLog("Recognizing");
                    if (runtime.milliseconds() - start_time > 6000) {
                        // Cannot locate beacon, retry approach
                        ADBLog("Cannot locate beacon, retry approach");
                        return false;
                    }
                } else if (bt.GetState() == Tracker.State.RECOGNIZED) {
                    ADBLog("Recognized");
                    isRight = bt.button().isRight;
                    located = true;
                    break;
//                        bt.SetState(Tracker.State.TRACKING);
                }
            }
            bt.Close();

            */
        if (!opModeIsActive()) return false;

         // Now press!
         if (btn.isRight) {
             ADBLog("============= Pushing right");
             rightPusher.setPower(1);
         } else {
             ADBLog("============= Pushing left");
             leftPusher.setPower(-1);
         }
        sleep(100);

        int Y = getRangeSensor();
        drive.DriveByEncoders(90 * dir, 0.2, Y + 90);
        drive.brake();

        if (btn.isRight) {
            rightPusher.setPower(-1);
        } else {
            leftPusher.setPower(1);
        }

        return true;
    }

    public boolean getVuforiaCoord(VuforiaTrackable target, VectorF coord, int timeout) {
        double start_time = runtime.milliseconds();

        while (runtime.milliseconds() - start_time < timeout) {
            OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getRawUpdatedPose();
            if (pose != null) {
                float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 16);
                coord.getData()[0] = poseData[3];
                coord.getData()[1] = poseData[11];
                return true;
            }
            sleep(5);
        }
        return false;
    }

    public int getRangeSensor() {
        byte[] array = rangeSensor.read(0x04, 2);
        //amplifies the y value to emphasize it in the vector (it is in centimeters so it needs to be increased)
        return array[0] * 10;
    }

    class TimestampedData {
        public double X;
        public double time;

        public TimestampedData(double x, double t) {
            X = x;
            time = t;
        }
    }

    class Integrator {
        private int delay = 0;
        private double S = 0;
        ArrayList<TimestampedData> history = new ArrayList<TimestampedData>();

        public Integrator(int d) {
            delay = d;
        }

        public void Add(double x, double t) {
            history.add(new TimestampedData(x, t));
            S += x;
        }

        public double Get(double t) {
            while (history.size() > 0) {
                if (history.get(0).time < t - delay) {
                    S -= history.get(0).X;
                    history.remove(0);
                }
                else break;
            }
            if (history.size() == 0) S = 0;
            return S;
        }

        public void Reset() {
            S = 0;
            history.clear();
        }
    }

    private Mat GetCameraImage() {
        Mat mat = null;
        while (mat == null) {
            try {
                VuforiaLocalizer.CloseableFrame frame = vuforia.getFrameQueue().take(); //takes the frame at the head of the queue
                Image rgb = null;
                long numImages = frame.getNumImages();
                for (int i = 0; i < numImages; i++) { //finds a frame that is in color, not grayscale
                    if (frame.getImage(i).getFormat() == PIXEL_FORMAT.RGB888) {
                        rgb = frame.getImage(i);
                        break;
                    }
                }
                if (rgb != null && rgb.getPixels() != null) {
                    ByteBuffer bb = rgb.getPixels();
                    byte[] b = new byte[bb.remaining()];
                    bb.get(b);

                    mat = new Mat(rgb.getBufferHeight(), rgb.getBufferWidth(), CvType.CV_8UC3);
                    mat.put(0, 0, b);

                    frame.close();

//                Imgproc.pyrDown(mat, mat);
                    Imgproc.resize(mat, mat, new Size(852, 480));
                    Core.flip(mat, mat, 1);

//                Bitmap bm = Bitmap.createBitmap(rgb.getWidth(), rgb.getHeight(), Bitmap.Config.ARGB_8888);
//                bm = bm.copy(Bitmap.Config.RGB_565, true);
//                bm.copyPixelsFromBuffer(rgb.getPixels());
                }
            } catch (InterruptedException ex) {
                sleep(10);
            }
        }
        return mat;
    }

    // Display image on phone's screen
    protected volatile static Bitmap mImageMap = null;
    private void DisplayImage(Mat img) {
        // Scale down x2
        Core.flip(img, img, -1);
        mImageMap = Bitmap.createBitmap(img.cols(), img.rows(), Bitmap.Config.ARGB_8888);
        Utils.matToBitmap(img, mImageMap);

        ((FtcRobotControllerActivity) hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {
                mImageView.setImageBitmap(mImageMap);
            }
        });
    }

    // Load OpenCV libraries
    static {
        System.loadLibrary("opencv_java3");
    }
}

