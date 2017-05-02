package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab.experimental;

import android.graphics.Bitmap;
import android.widget.ImageView;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;
import com.vuforia.HINT;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.opmodes.sandstoRM.BetterDarudeAutoNav;
import org.firstinspires.ftc.rmrobotics.util.autonav.AutoNavConfig;
import org.firstinspires.ftc.rmrobotics.util.autonav.Drive2;
import org.firstinspires.ftc.rmrobotics.util.autonav.vision.BeaconRecognizer;
import org.firstinspires.ftc.rmrobotics.util.autonav.vision.BeaconRecognizerAlternate;
import org.firstinspires.ftc.rmrobotics.util.autonav.vision.ButtonFinder;
import org.firstinspires.ftc.rmrobotics.util.autonav.vision.RMVuforia;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;
import org.firstinspires.ftc.rmrobotics.util.enums.Direction;
import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.opencv.android.Utils;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Created by Simon on 4/13/2017.
 */

@Autonomous(name = "AutoExperimental")
@Disabled
public class AutoExperimental extends FeRMiLinear{

    //runtime calculations
    static ElapsedTime runtime = new ElapsedTime();

    private RMVuforia vuforia;

    private Drive2 drive = null;

    private ImageView mImageView;

    private double dir = 1;

    private final int RECOGNITION_DIST_Y = 330; // Distance to turn on OpenCV
    private final int RECOGNITION_TOLER_Y = 40; // Positioning tolerance along Y
    private final int RECOGNITION_TOLER_X = 30; // Positioning tolerance along Y
    private final int REPOSITION_DIST = 480; // Distance to back up if out of position to recognize button

    @Override
    public void runOpMode() {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.BACKWARD);

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

        List<VuforiaTrackable> allTrackables = new ArrayList<VuforiaTrackable>();
        allTrackables.addAll(targets);

        // Create NAVX device
        DeviceInterfaceModule dim = hardwareMap.deviceInterfaceModule.get("dim");
        ADBLog(dim.toString());

        ADBLog("NavX is ready");

        //drive class for big roboto
        FR.setDirection(DcMotorSimple.Direction.FORWARD);
        BR.setDirection(DcMotorSimple.Direction.FORWARD);
        drive = new Drive2(
                FL,
                FR,
                BL,
                BR,
                navx,
                telemetry,
                this
        );
        drive.ReverseDirection();

        targets.activate();
        float x = 0;
        float y = 0;
        Mat img = null;

        sleep(500);
        ADBLog("Initialization complete");
        telemetry.clear();
        telemetry.addData("Finished", "the initialization");
        telemetry.update();
//            waitForStart();
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////
// Start here
//             Start the engine!!!
//             1 encoder tick ~ 1 mm

//            test(cfg);

        try {

            ADBLogReset();
            ADBLog("Start");
            new Thread(drive).start();

            flyL.setPower(1);
            flyR.setPower(1);
            drive.DriveByEncoders(0, 0.4, 335);
            drive.brake();
            sleep(1000);
            index.setPosition(1.1); // TODO: check position
            sleep(760);
            index.setPosition(.5); // TODO: check position
            sleep(1000);
            index.setPosition(1.1); // TODO: check position
            sleep(1500);
            index.setPosition(.5); // TODO: check position
            flyL.setPower(0);
            flyR.setPower(0);

            ADBLog("Step 1");
            drive.DriveByEncoders(63 * dir, 0.4, 590);
            drive.DriveByEncoders(63 * dir, 0.5, 750);
            drive.DriveByEncoders(63 * dir, 0.5, 100);
            ADBLog("Step 2");
            drive.DriveByEncoders(0 * dir, 0.5, 300);
            drive.DriveByEncoders(0 * dir, 0.4, 100);
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


///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////
            // Go to the second beacon
            ADBLog("Step 1-2");
            drive.DriveByEncoders(90 * dir, -0.3, 100);
            ADBLog("Step 2-2");
            drive.DriveByEncoders(172 * dir, -0.4, 400);
            drive.DriveByEncoders(172 * dir, -0.5, 500);
            drive.DriveByEncoders(172 * dir, -0.5, 1070);
            drive.brake();
            ADBLog("Step 3-2");
            drive.TurnToAngle(90 * dir);
            ADBLog("Step 5");

            pushLeft.setPower(0);
            pushRight.setPower(0);

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

            drive.DriveByEncoders(90 * dir, -0.2, 100);
            drive.brake();
            sleep(50);
            pushLeft.setPower(0);
            pushRight.setPower(0);
        } finally {
            ADBLog("Exiting opmode");
            drive.Stop();
            pushLeft.setPower(0);
            pushLeft.close();
            pushRight.setPower(0);
            pushRight.close();
            index.setPosition(0.5); // TODO: check position
            index.close();
            navx.close();
            stop();
        }
    }

    private void test(AutoNavConfig c) {
        telemetry.addData("cfg", c.isRed + " " + c.firstBeacon + " " + c.isRight + " " + c.secondBeacon);
        telemetry.update();

        ButtonFinder.EllipseLocationResult btn = null;
        BeaconRecognizerAlternate br = new BeaconRecognizerAlternate();

        double start_time = runtime.milliseconds();

        while(opModeIsActive()) {
            ADBLog("Recognizing");
            Mat img = GetCameraImage();
            img = img.t();
            boolean isRed = false;
            btn = br.detectButtons(img, isRed);
            img = img.t();
            DisplayImage(img);
            if (btn != null) {
                ADBLog("Recognized, got button");
                break;
            }
            ADBLog("Cannot find button in the image");
            sleep(10);
            if (runtime.milliseconds() - start_time > 1000) {
                ADBLog("Cannot locate beacon, retry approach");
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
        byte[] array = rangeReader.read(0x04, 2);
        //amplifies the y value to emphasize it in the vector (it is in centimeters so it needs to be increased)
        return array[0] * 10;
    }

    BetterDarudeAutoNav.Integrator I = new BetterDarudeAutoNav.Integrator(400);
    double prevX = 0;
    double prevY = 0;
    double prevT = runtime.milliseconds();

    private double Approach(VuforiaTrackable target) {
        double X = 0;
        double Y = 1000;

        // First find target
        VectorF vuf_coord = new VectorF(0, 0);
        while (!getVuforiaCoord(target, vuf_coord, 100)) {
            ADBLog("Lost Vurforia track. Do something!!!");
            sleep(30);
        }

        X = prevX = vuf_coord.getData()[0] - 10;
        Y = prevY = getRangeSensor();
        drive.resetYDist();
        I.Reset();
        boolean retesting = false;
        ADBLog("Intercepted. Coords: X:" + X + ", Y:" + Y);

        // Follow perpendicular
        while (opModeIsActive()) {
            if (((VuforiaTrackableDefaultListener) target.getListener()).isVisible()) {
                if (!getVuforiaCoord(target, vuf_coord, 100)) {
                    // Lost Vuforia track! Do something!
                    ADBLog("Lost Vurforia track. Do something!!!");
                    continue;
                }

                X = (vuf_coord.getData()[0] + 20);
                Y = getRangeSensor();

                double currT = runtime.milliseconds();
                // Get incremental data from drive
                double Xd = drive.getYDistIncr();

                I.Add(Xd, currT);

                double xi = 1.1 * I.Get(currT);
                double XE = X - xi;
                double absXE = Math.abs(XE);
                double XV = (prevX - X) / (currT - prevT);

                double YE = Y - RECOGNITION_DIST_Y;
                double absYE = Math.abs(YE);
                double YV = (prevY - Y) / (currT - prevT);

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

                ADBLog("x:" + X + ", xi:" + xi + ", y:" + Y + ", XE:" + XE + ", YE:" + YE + ", XV: " + XV + ", YV:" + YV + ", xd:" + x_dir + ", yd:" + y_dir + ", sp:" + sp);
                drive.VecDriveBalanced(y_dir, x_dir, sp, 2000);


                prevX = X;
                prevY = Y;
                prevT = currT;
            }
        }

        ADBLog("Brake");
        drive.brake();

        return X;
    }

    /*
    private double Approach(VuforiaTrackable target) {
        double X = 0;
        double Y = 1000;

        // First find target
        VectorF vuf_coord = new VectorF(0, 0);
        while (!getVuforiaCoord(target, vuf_coord, 100)) {
            ADBLog("Lost Vurforia track. Do something!!!");
            sleep(30);
        }

        X = prevX = vuf_coord.getData()[0] - 10;
        Y = prevY = getRangeSensor();
        drive.resetYDist();
        I.Reset();
        boolean retesting = false;
        ADBLog("Intercepted. Coords: X:" + X + ", Y:" + Y);

        // Follow perpendicular
        while (opModeIsActive()) {
            if (((VuforiaTrackableDefaultListener) target.getListener()).isVisible()) {
                if (!getVuforiaCoord(target, vuf_coord, 100)) {
                    // Lost Vuforia track! Do something!
                    ADBLog("Lost Vurforia track. Do something!!!");
                    continue;
                }

                X = (vuf_coord.getData()[0] + 20);
                Y = getRangeSensor();

                double currT = runtime.milliseconds();
                // Get incremental data from drive
                double Xd = drive.getYDistIncr();

                I.Add(Xd, currT);

                double xi = 1.5 * I.Get(currT);
                double XE = X - xi;
                double absXE = Math.abs(XE);
                double XV = (prevX - X) / (currT - prevT);

                double YE = Y - RECOGNITION_DIST_Y;
                double absYE = Math.abs(YE);
                double YV = (prevY - Y) / (currT - prevT);

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
                    if (Math.signum(X) != Math.signum(XE)) {
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

                ADBLog("x:" + X + ", xi:" + xi + ", y:" + Y + ", XE:" + XE + ", YE:" + YE + ", XV: " + XV + ", YV:" + YV + ", xd:" + x_dir + ", yd:" + y_dir + ", sp:" + sp);
                drive.VecDriveBalanced(y_dir, x_dir, sp, 2000);


                prevX = X;
                prevY = Y;
                prevT = currT;
            }
        }

        ADBLog("Brake");
        drive.brake();

        return X;
    }*/


    private boolean RecognizeAndPush(boolean isRed) {
        ButtonFinder.EllipseLocationResult btn = null;
        BeaconRecognizer br = new BeaconRecognizer();

        double start_time = runtime.milliseconds();

        while(opModeIsActive()) {
            ADBLog("Recognizing");
            Mat img = GetCameraImage();
            img = img.t();
            btn = br.detectButtons(img, isRed);
            img = img.t();
            DisplayImage(img);
            if (btn != null) {
                ADBLog("Recognized, got button");
                break;
            }
            ADBLog("Cannot find button in the image");
            sleep(10);
            if (runtime.milliseconds() - start_time > 1000) {
                ADBLog("Cannot locate beacon, retry approach");
                return false;
            }
        }
        if(btn == null) return false;

        if (!opModeIsActive()) return false;

        // Now push!
        if (btn.isRight) {
            ADBLog("============= Pushing right");
            pushRight.setPower(1); // TODO: check direction
        } else {
            ADBLog("============= Pushing left");
            pushLeft.setPower(-1); // TODO: check direction
        }
        sleep(100);

        int Y = getRangeSensor();
        drive.DriveByEncoders(90 * dir, 0.2, Y + 180);
        drive.brake();
        sleep(100);

        if (btn.isRight) {
            pushRight.setPower(1);
        } else {
            pushLeft.setPower(1);
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
        byte[] array = rangeReader.read(0x04, 2);
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
        ArrayList<BetterDarudeAutoNav.TimestampedData> history = new ArrayList<BetterDarudeAutoNav.TimestampedData>();

        public Integrator(int d) {
            delay = d;
        }

        public void Add(double x, double t) {
            history.add(new BetterDarudeAutoNav.TimestampedData(x, t));
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

                    Imgproc.resize(mat, mat, new Size(852, 480));
                    Core.flip(mat, mat, 1);
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