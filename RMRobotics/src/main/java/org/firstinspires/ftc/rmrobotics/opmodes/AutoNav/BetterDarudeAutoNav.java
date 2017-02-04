package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;


import android.content.Context;
import android.graphics.Bitmap;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.util.JsonReader;
import android.util.JsonWriter;
import android.util.Log;
import android.view.SurfaceView;
import android.view.ViewDebug;
import android.widget.ImageView;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.ftccommon.DbgLog;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.hardware.GyroSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.Servo;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.rmrobotics.R;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.io.FileNotFoundException;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.nio.ByteBuffer;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import org.firstinspires.ftc.rmrobotics.opmodes.AutoNav.Drive;
import org.firstinspires.ftc.robotcore.internal.VuforiaLocalizerImpl;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import pm.BeaconRecognizer;
import pm.ButtonFinder;

import static java.lang.System.out;


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
            if(cfg.isRight) dir = 1;
            else dir = -1;

            //init vuforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);
            parameters.vuforiaLicenseKey = "AfMO3Zb/////AAAAGRP6yrVXWkBalSEp+NZax44NTzFOkclxZo99uF0DhcZXRp0O2qHl6wcPx8Bp+3dppnmePU1HwDeEwSmWl5k/QEHiTAXMEfc/DBjkAVcMvEBnJGaGEHjKsD4/YaU+cdVcU7Q+NbG7fT1KYcpRUu2btnjhHWCPPhS0mu2AtWNw87FuXX0ob8GCP9jc7fKqVkdgChkshd9aaqJ6113IosFisru2Jk2V098iWv20c3ASuEp0oZfFp6DtLKjx0GQWc7+PhM8rxNRRFNbvwTKAUeiQBRyDoh+UroSO39fon2BcFrStL6pEKa6pqIahZGaeIZXcbx9KpmfO6LNX2vkIgdbw+KZjcszjI0x1RYZDcCu4U5V6";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;
            parameters.useExtendedTracking = false;

            vuforia = new RMVuforia(parameters);
            Vuforia.setHint(HINT.HINT_MAX_SIMULTANEOUS_IMAGE_TARGETS, 4);
            VuforiaTrackables targets = this.vuforia.loadTrackablesFromAsset("FTC_2016-17");

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
            //currently only using one since only going to one beacon
            for(int it = 0; it < 4; it++) {
                if(targets.get(it).getName().equals(cfg.firstBeacon)) firstTarget = targets.get(it);
                if(targets.get(it).getName().equals(cfg.secondBeacon)) secondTarget = targets.get(it);
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

            DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get("dim");;
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
            if(TestDriveTrain) {

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
                if(true) return;

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


            new Thread(drive).start();

            drive.resetDistance();
            drive.DriveByEncoders(0, 0.2, 150);
            drive.TurnToAngle(30*dir);
            sleep(1000);
            drive.resetDistance();
            drive.DriveByEncoders(25*dir,0.4,310);
            drive.resetDistance();
            drive.DriveByEncoders(22*dir,0.2,500);
            drive.resetDistance();
            drive.DriveByEncoders(20*dir,0.1,310);
            drive.TurnToAngle(90*dir);
            drive.brake();
            sleep(3000);

            // Approaching first target
            double d = Approach(firstTarget);

            // Stop Vuforia
            ADBLog("Pausing Vuforia");
            vuforia.RMPause();

            if(!opModeIsActive()) return;

            boolean start;
            boolean located = false;
            // OpenCV beacon tracking class
            BeaconTracker bt = null;
            // here goes code for poking the button via image recognition
            try {
                // Start OpenCV
                // Starting video stream
                bt = new BeaconTracker(
                        (CameraBridgeViewBase)((FtcRobotControllerActivity)hardwareMap.appContext).findViewById(R.id.camera_view),
                        this,
                        cfg.isRed);
                ADBLog("Created OpenCV");
                //first runthru variable
                start = true;
                int verify = 0;
                boolean isRight = false;
                while (opModeIsActive()) {
                    if (bt.GetState() == Tracker.State.RECOGNIZING) {
                        //trying to find button
                        sleep(30);
                        ADBLog("Recognizing");
                    } else if (bt.GetState() == Tracker.State.RECOGNIZED) {
                        ADBLog("Recognized. Starting approach.");
                        isRight = bt.button().isRight;
                        located = true;
                        break;
//                        bt.SetState(Tracker.State.TRACKING);
                    } else if(bt.GetState() == Tracker.State.LOST) {
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
                bt.Close();
                if(!opModeIsActive()) return;

                if(located) {
                    // Now press!
                    if (isRight) {
                        ADBLog("Pushing right");
                        rightPusher.setPower(1);
                        sleep(800);
                        rightPusher.setPower(0);
                    } else {
                        ADBLog("Pushing left");
                        leftPusher.setPower(-1);
                        sleep(800);
                        leftPusher.setPower(0);
                    }
                }
            } catch(Exception ex) {
                ADBLog("Exception!: " + ex.getMessage());
                if(bt != null) bt.Close();
            }

            drive.resetDistance();
            drive.DriveByEncoders(90*dir,0.1,330);
            sleep(4000);

            if (!opModeIsActive()) {
                drive.Stop();
                navx_device.close();
                sleep(500);
                stop();
                return;
            }

            vuforia.RMResume();

            drive.resetDistance();
            drive.DriveByEncoders(0, -0.2, 400);
            drive.TurnToAngle(0);
            sleep(1000);
            drive.resetDistance();
            drive.DriveByEncoders(0,0.2,1100);
            drive.TurnToAngle(90*dir);
            drive.brake();
            sleep(2000);

            // Approaching first target
            d = Approach(secondTarget);

            // Stop Vuforia
            ADBLog("Pausing Vuforia");
            vuforia.RMPause();

            if(!opModeIsActive()) return;

            located = false;
            // OpenCV beacon tracking class
            bt = null;
            // here goes code for poking the button via image recognition
            try {
                // Start OpenCV
                // Starting video stream
                bt = new BeaconTracker(
                        (CameraBridgeViewBase)((FtcRobotControllerActivity)hardwareMap.appContext).findViewById(R.id.camera_view),
                        this,
                        cfg.isRed);
                ADBLog("Created OpenCV");
                //first runthru variable
                start = true;
                int verify = 0;
                boolean isRight = false;
                while (opModeIsActive()) {
                    if (bt.GetState() == Tracker.State.RECOGNIZING) {
                        //trying to find button
                        sleep(30);
                        ADBLog("Recognizing");
                    } else if (bt.GetState() == Tracker.State.RECOGNIZED) {
                        ADBLog("Recognized. Starting approach.");
                        isRight = bt.button().isRight;
                        located = true;
                        break;
//                        bt.SetState(Tracker.State.TRACKING);
                    } else if(bt.GetState() == Tracker.State.LOST) {
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
                bt.Close();
                if(!opModeIsActive()) return;

                if(located) {
                    // Now press!
                    if (isRight) {
                        ADBLog("Pushing right");
                        rightPusher.setPower(1);
                        sleep(800);
                        rightPusher.setPower(0);
                    } else {
                        ADBLog("Pushing left");
                        leftPusher.setPower(-1);
                        sleep(800);
                        leftPusher.setPower(0);
                    }
                }
            } catch(Exception ex) {
                ADBLog("Exception!: " + ex.getMessage());
                if(bt != null) bt.Close();
            }

            drive.resetDistance();
            drive.DriveByEncoders(90*dir,0.1,330);
            sleep(4000);




            if (true) {
                drive.Stop();
                navx_device.close();
                stop();
                return;
            }










        }
    }

    static public void ADBLog(String msg) {
        RobotLog.d(String.format("%.0f", runtime.milliseconds()) + ": " + msg);
    }

    static final double Y_DIST = 150;
    static final double X_TOLERANCE = 20;

    enum States {OFF_LEFT, OFF_RIGHT, ON_TARGET};
    private double Approach(VuforiaTrackable target)
    {
        double x = 0;
        double y = 1000;
        double offset = 0;
        int angle = 0;
        int prevAngle = 0;
        double d = 1000;
        double head = 90*dir;

        // First intercept perpendicular line
        while (opModeIsActive())
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

                    int dist = (int)Math.abs(Math.sqrt(2)*d);

                    ADBLog("Driving for: " + dist);
                    if(d < 0) {
                        drive.TurnToAngle(-60 + head);
                        drive.resetDistance();
                        drive.DriveByEncoders(-60 + head, 0.05, dist);
                    } else {
                        drive.TurnToAngle(60 + head);
                        drive.resetDistance();
                        drive.DriveByEncoders(60 + head, 0.05, dist);
                    }
                    drive.brake();
                    drive.TurnToAngle(head);
                    sleep(3000);
                    break;
                } else {
                    //continue;
                    ADBLog("Continue");
                }
            } else {
                ADBLog("Lost beacon. What to do?");
            }
            sleep(10);
        }

        ADBLog("Intercepted");


//        States state = States.ON_TARGET;

        // Follow perpendicular
        while (opModeIsActive() && y > 330)
        {
            if(((VuforiaTrackableDefaultListener) target.getListener()).isVisible()) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getRawUpdatedPose();
                if (pose != null) {
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 16);
                    x = poseData[3] - 10;
                    //y = poseData[11];

                    byte[] array = rangeSensor.read(0x04, 2);
                    //amplifies the y value to emphasize it in the vector (it is in centimeters so it needs to be increased)
                    y = array[0] * 10;



                    if (y < 330) {
                        drive.VecDrive(0, 0, 0, 1000);
                        break;
                    }

                    double sp = 0.4;
                    //if (Math.abs(y - 420) < 100) sp = 0.2;

                    ADBLog("Coords: x= " + x + ", y= " + (y -330) + ", sp=" + sp);

                    drive.VecDriveBalanced(y - 330, -x, sp, 100);
                }
                sleep(30);
            }
        }

/*
        double steering_angle = 15;
        while (opModeIsActive() && y > 350)
        {
            if(((VuforiaTrackableDefaultListener) target.getListener()).isVisible()) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) target.getListener()).getRawUpdatedPose();
                if (pose != null) {
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 16);
                    x = poseData[3] - 30;
                    y = poseData[11];

                    if (y < 350) break;

                    double b = Math.atan(x / y);
                    double a = Math.toRadians(navx_device.getYaw());

                    double r = Math.sqrt(x * x + y * y);
                    x = r * Math.sin(b + a);
                    offset = 170 * Math.sin(a);
                    ADBLog("Angles: a= " + a / Math.PI * 180 + ", b= " + b / Math.PI * 180);
                    ADBLog("Coords: x'= " + x + ", y'= " + y + ", r=" + r + ", x= " + x + ", offset=" + offset);

                    drive.VecDrive(10, 0, 0.07, 10000);

                    d = x + offset;

                    ADBLog("Total offset: " + d);
                    switch (state) {
                        case ON_TARGET:
                            if (Math.abs(d) > X_TOLERANCE) {
                                if (d < 0) {
                                    state = States.OFF_RIGHT;
                                    drive.TurnToAngle(-steering_angle);
                                    ADBLog("Angle: " + (-steering_angle));
                                } else {
                                    state = States.OFF_LEFT;
                                    drive.TurnToAngle(steering_angle);
                                    ADBLog("Angle: " + steering_angle);
                                }
                            } else state = States.ON_TARGET;
                            break;
                        case OFF_RIGHT:
                            if (d > 0) {
                                drive.TurnToAngle(0);
                                state = States.ON_TARGET;
                                steering_angle /=2;
                                ADBLog("Angle: 0");
                            }
                            break;
                        case OFF_LEFT:
                            if (d < 0) {
                                drive.TurnToAngle(0);
                                state = States.ON_TARGET;
                                steering_angle /=2;
                                ADBLog("Angle: 0");
                            }
                            break;
                    }
                } else {
//                ADBLog("Lost beacon");
//                drive.brake();
//                continue;
                }
                sleep(30);
            } else {
                ADBLog("Lost beacon. Do something now!");
                sleep(10);
            }
        } */

        drive.TurnToAngle(head);

        sleep(1000);


        ADBLog("Brake");
        drive.brake();

        return x;
    }

    // Load OpenCV libraries
    static {
        System.loadLibrary("opencv_java3");
    }
}

