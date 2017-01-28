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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.GyroSensor;

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

class RMVuforia extends VuforiaLocalizerImpl {
    RMVuforia(VuforiaLocalizer.Parameters parameters) {
        super(parameters);
    }

    // Pause and release camera
    public void RMPause() {
        pauseAR();
    }

    // Resume Vuforia
    public void RMResume() {
        resumeAR();
    }
}

@Autonomous(name = "DarudeAutoNav", group = "AutoNav")
public class DarudeAutoNav extends LinearOpMode {

    //runtime calculations
    static ElapsedTime runtime = new ElapsedTime();

    RMVuforia vuforia;

    //motors
    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    //servo
    Servo press;

    //navx
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private ImageView mImageView;

    private static volatile boolean teamIsRed = true;
    private static volatile boolean redIsLeft;


    @Override
    public void runOpMode() throws InterruptedException {
        {
            // Read config file
            AutoNavConfig cfg = new AutoNavConfig();
            cfg.ReadConfig(((FtcRobotControllerActivity) hardwareMap.appContext));

            //init vuforia
            VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(/*com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId*/);
            parameters.vuforiaLicenseKey = "AfMO3Zb/////AAAAGRP6yrVXWkBalSEp+NZax44NTzFOkclxZo99uF0DhcZXRp0O2qHl6wcPx8Bp+3dppnmePU1HwDeEwSmWl5k/QEHiTAXMEfc/DBjkAVcMvEBnJGaGEHjKsD4/YaU+cdVcU7Q+NbG7fT1KYcpRUu2btnjhHWCPPhS0mu2AtWNw87FuXX0ob8GCP9jc7fKqVkdgChkshd9aaqJ6113IosFisru2Jk2V098iWv20c3ASuEp0oZfFp6DtLKjx0GQWc7+PhM8rxNRRFNbvwTKAUeiQBRyDoh+UroSO39fon2BcFrStL6pEKa6pqIahZGaeIZXcbx9KpmfO6LNX2vkIgdbw+KZjcszjI0x1RYZDcCu4U5V6";
            parameters.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
            parameters.cameraMonitorFeedback = VuforiaLocalizer.Parameters.CameraMonitorFeedback.AXES;

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
            press = hardwareMap.servo.get("press");

            // Create NAVX device
            navx_device = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                    NAVX_DIM_I2C_PORT,
                    AHRS.DeviceDataType.kProcessedData,
                    NAVX_DEVICE_UPDATE_RATE_HZ);

            // Create optical sensor
            I2cDeviceSynch rangeSensor = new I2cDeviceSynchImpl(
                    hardwareMap.i2cDevice.get("range"),
                    I2cAddr.create8bit(0x62), // Sensor I2C address
                    false);
            rangeSensor.engage();

            Drive2 drive = new Drive2(
                    frontLeft,
                    frontRight,
                    backLeft,
                    backRight,
                    navx_device,
                    telemetry
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

                press.setPosition(0.5);
                stop();
                if(true) return;

            } else {
                new Thread(drive).start();
            }



            // Initialize display view
//            mImageView = (ImageView) ((FtcRobotControllerActivity) hardwareMap.appContext).findViewById(com.qualcomm.ftcrobotcontroller.R.id.imageViewId);


            press.setPosition(0.2);

            targets.activate();
            float x = 0;
            float y = 0;
            Mat img = null;

            sleep(500);
            ADBLog("Initializtino complete");
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

//            drive.VecDrive(0, 100, 0.4, 0, 1, 5000);
//            sleep(1000);
//

//            sleep(100);
//            drive.VecDrive(0, 200, .6, 0, 1, 2000);
//            sleep(2000);
//
//            if(true)
//            {
//                drive.Stop();
//                navx_device.close();
//                stop();
//                return;
//            }


            // Moving fast to position for beacon detection
            drive.VecDrive(200, 100, .3, 1000);
            sleep(1000);

            // Continue slowly checking weather Vuforia locked yet
            drive.VecDrive(200, 100, .13, 8000);

            boolean ec = true;
            while (ec && opModeIsActive()) {
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) firstTarget.getListener()).getRawPose();
                    if (pose != null) {
                        ec = false;
                        ADBLog("Vuforia locked!");
                    } else {
                        sleep(5);
                    }
//                for (VuforiaTrackable trackable : allTrackables) {
//                    OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRawPose();
//                    if (pose != null) {
//                        ec = false;
//                        ADBLog("Vuforia locked!");
//                    }
//                }
            }
            if (!opModeIsActive()) {
                drive.Stop();
                navx_device.close();
                stop();
                return;
            }
            x = y = 0;
            // Vuforia aproach to a position to recognize beacon
            ADBLog("Begin Vuforia approach");
            while (opModeIsActive()) {
                telemetry.clear();
                OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) firstTarget.getListener()).getRawUpdatedPose();
                if (pose != null) {
                    float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                    telemetry.addData("x", poseData[7]);
                    telemetry.addData("y", poseData[11]);
                    x = poseData[7];
                    y = poseData[11];

                    ADBLog("Vuforia coords: x=" + x + ", y=" + y);

//                    if(Math.abs(y-420) < 100) sp = 0.1;

                    drive.VecDrive(-x/3, y - 430, 0.2, 100);

                    if (x < 40 && x > -40 && y > 380 && y < 470) {
                        drive.VecDrive(0, 0, 0, 1000);
                        break;
                    }
                } else {
                    sleep(1);
                }
            }
            if (!opModeIsActive()) {
                drive.Stop();
                navx_device.close();
                stop();
                return;
            }

            // Stop Vuforia
            ADBLog("Pausing Vuforia");
            vuforia.RMPause();

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

                boolean start = true;
                int verify = 0;
                while (opModeIsActive()) {
                    if (bt.GetState() == Tracker.State.RECOGNIZING) {
                        sleep(30);
                        ADBLog("Recognizing");
                    } else if (bt.GetState() == Tracker.State.RECOGNIZED) {
                        ADBLog("Recognized. Starting approach.");
                        bt.SetState(Tracker.State.TRACKING);
                    } else if(bt.GetState() == Tracker.State.LOST) {
                        // Lost button. Todo try to recover!!!!
                        sleep(5);
                        ADBLog("Lost button.");
                        continue;
                    } else if (bt.GetState() == Tracker.State.TRACKING) {
                        RotatedRect btn = bt.getBtnPosition();
                        if (btn == null) {
                            sleep(5); // No updates yet, wait
                            continue;
                        }

                        byte[] array = rangeSensor.read(0x04, 2);
                        y = array[0] * 10;

                        double Xb = btn.center.y - 250;
                        ADBLog("Position OpenCV X: " + Double.toString(Xb) + ", Sensor: " + Double.toString(y));

                        if (y > 260) {
                            Xb /= 6;
                            if(start) {
                                start = false;
                                drive.VecDrive(-Xb, y - 210, 0.3, 200);
                                sleep(50);
                            }
                            drive.VecDrive(-Xb, y - 210, 0.17, 200);
                        } else if (y > 240) {
                            Xb /= 6;
                            drive.VecDrive(-Xb, y - 210, 0.1, 200);
                        } else {
                            Xb /= 8;
                            if (Xb < 15 && Xb > -8 && y < 250 && y > 190) {
                                drive.VecDrive(0, 0, 0, 2000);
                                verify++;
                                if (verify > 3) {
                                    bt.Close();
                                    press.setPosition(.43);
                                    break;
                                }
                            } else {
                                drive.VecDrive(0, 0, 0.0, 200);
                                sleep(20);
                                drive.VecDrive(-Xb, y - 210, 0.6, 200);
                                sleep(50);
                                drive.VecDrive(-Xb, y - 210, 0.3, 150);
                                sleep(250);
                                verify = 0;
                            }
                        }
                    }
                }
                bt.Close();
            } catch(Exception ex) {
                ADBLog("Exception!: " + ex.getMessage());
                if(bt != null) bt.Close();
            }

            if (!opModeIsActive()) {
                drive.Stop();
                navx_device.close();
                sleep(500);
                stop();
                return;
            }

            ADBLog("Press!");
            sleep(100);
            drive.VecDrive(9, 20, 1, 500);
            sleep(50);
            drive.VecDrive(4, 20, 0.6, 700);
            sleep(300);

            bt.Close();
            drive.Stop();
            navx_device.close();
            sleep(500);
            stop();
            boolean debug = true;
            if (debug) return;

/*
                img = GetCameraImage();
//            boolean redOnLeft = br.RedOnTheLeft(img);
                boolean redOnLeft = false;

                // Mat img, boolean redIsLeft, boolean getRed)
//            ButtonFinder.EllipseLocationResult btn0 = br.detectButtons(img, redOnLeft, true);
//            DisplayImage(img);

                boolean found = false;
                while (!found) {
                    img = GetCameraImage();
                    ButtonFinder.EllipseLocationResult btn0 = br.detectButtons(img, redOnLeft, true);
                    DisplayImage(img);
                    if (btn0 != null) found = true;
                    img = GetCameraImage();
                }

                //0 forward 90 right
                while (opModeIsActive()) {
                    for (VuforiaTrackable trackable : allTrackables) {
                        OpenGLMatrix pose = ((VuforiaTrackableDefaultListener) trackable.getListener()).getRawPose();
                        if (pose != null) {
                            float[] poseData = Arrays.copyOfRange(pose.transposed().getData(), 0, 12);
                            telemetry.addData("x", poseData[7]);
                            telemetry.addData("y", poseData[11]);
                            x = poseData[7];
                            y = poseData[11];
                        }
                    }

                    if (x < 0.1) x = 120;

                    RotatedRect btn = null;
                    sleep(300);
                    while (btn == null) {
                        img = GetCameraImage();
                        btn = br.TrackButton(img);
                        DisplayImage(img);
                        if (btn != null) break;
                    }

                    double Xb = btn.center.y - 250;
                    double angle = CalculateAngle((float) Xb, y - 180);
                    double speed = CalculateSpeed((float) Xb, y - 180, .6);
                    telemetry.addData("Xb", Xb);
                    telemetry.addData("angle", angle);
                    telemetry.addData("speed", speed);
                    Log.d("Position", Double.toString(Xb) + " X-Image " + Float.toString(x) + " X-Vuforia " + Float.toString(y) + " Y-Vuforia ");
                    telemetry.update();

                    if (y < 300) speed /= 2;
                    else speed /= 1.3;

                    drive.driveDirection(angle, speed, 200);
                    if (Xb < 20 && Xb > -20 && y < 200 && y > 160) {
                        drive.brake();
                        break;
                    }
                }

                if (!opModeIsActive()) stop();
            }

            press.setPosition(.405);
            sleep(500);
            drive.driveDirection(90, .4, 450);

            sleep(1000); */
//            drive.brake();
        }
    }

    /*
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
*/


    static public void ADBLog(String msg) {
        RobotLog.d(String.format("%.0f", runtime.milliseconds()) + ": " + msg);
    }

    // Load OpenCV libraries
    static {
        System.loadLibrary("opencv_java3");
    }
}

class AutoNavConfig {
    public boolean isRed = false;
    public String firstBeacon = "";
    public String secondBeacon = "";

    public void ReadConfig(FtcRobotControllerActivity act) {
        try {
            InputStream in = act.openFileInput("DarudeAutoNavCfg");
            JsonReader reader = new JsonReader(new InputStreamReader(in, "UTF-8"));
            reader.beginObject();
            reader.nextName();
            isRed = reader.nextBoolean();
            reader.nextName();
            firstBeacon = reader.nextString();
            reader.nextName();
            secondBeacon = reader.nextString();
            reader.endObject();
            reader.close();
        } catch (FileNotFoundException ex) {
            RobotLog.d("Cannot create config file, creating default");
            // Create default file
            firstBeacon = "Wheels";
            secondBeacon = "Tools";
            WriteConfig(act);
        } catch (IOException ex) {
            RobotLog.d("Cannot create config file");
        }
    }

    public void WriteConfig(FtcRobotControllerActivity act) {
        try {
            OutputStream out = act.openFileOutput("DarudeAutoNavCfg",0);
            JsonWriter writer = new JsonWriter(new OutputStreamWriter(out, "UTF-8"));
            writer.setIndent("  ");
            writer.beginObject();
            writer.name("red").value(isRed);
            writer.name("firstBeacon").value(firstBeacon);
            writer.name("secondBeacon").value(secondBeacon);
            writer.endObject();
            writer.close();
        } catch (FileNotFoundException ex) {
            RobotLog.d("Cannot create config file");
        } catch (IOException ex) {
            RobotLog.d("Cannot create config file");
        }
    }
}

