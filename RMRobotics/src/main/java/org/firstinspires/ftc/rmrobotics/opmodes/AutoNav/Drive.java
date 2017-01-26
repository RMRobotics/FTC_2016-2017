package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.vuforia.Vec2F;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.opencv.core.Mat;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.Stack;

import static java.lang.Thread.sleep;


public class Drive implements Runnable {

    //runtime calculations
    ElapsedTime runtime = new ElapsedTime();

    //navx
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    Telemetry telemetry;

    //PID Variables
    private navXPIDController yawPIDController;
    private navXPIDController.PIDResult yawPIDResult;
    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1;
    private final double YAW_PID_P = 0.02;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    private int GYRO_DEVICE_TIMEOUT_MS = 500;

    private final int TN = 3; // Number of transition steps when changing direction
    private final double TRAN_STEP_TIME = 15; // Duration of transition step
    private final double MINX = 20; // Stop if within this distance from 0
    private final double MINY = 20; // Stop if within this distance from 0


    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;

    Drive(
            DcMotor fl,
            DcMotor fr,
            DcMotor bl,
            DcMotor br,
            AHRS nx,
            Telemetry tl) {
        //init motors
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        navx_device = nx;
        telemetry = tl;

        //set zero behavior and reverse motors so all are in the same direction.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        //enable encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Thread.yield();

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        boolean calibration_complete = false;
        while ( !calibration_complete ) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
            }
        }
        navx_device.zeroYaw();

        //initalize a PID controller
        yawPIDController = new navXPIDController( navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE/2, MAX_MOTOR_OUTPUT_VALUE/2);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
        yawPIDResult = new navXPIDController.PIDResult();

    }
    private volatile VectorF currentVector = new VectorF(0,0);
    private volatile VectorF goalVector = new VectorF(0,0);
    private volatile Stack<VectorF> stackVector = new Stack<>();
    private volatile double lastCommandTime = 0;
    private volatile double driveDuration = 0;
    private volatile double speed = 0;
    private volatile double minPower = 0;
    private volatile double maxPower = 0;

    public void VecDrive(double x, double y, double sp, double minPow, double maxPow, int maxDuration)
    {
        if(Math.abs(x) < MINX && Math.abs(y) < MINY) {
            x = 0;
            y = 0;
        }

        double fraction = 1.0/TN;
        synchronized (stackVector) {
            //creates a goal vector based on the current direction to 0,0
            goalVector = new VectorF((float)(x),(float)(y));
            //subtracts current vector from goal vector, creating a vector that points somewhere
            VectorF delVector = goalVector.subtracted(currentVector);
            //num subdivisions is xd can be changed to fine tune

            stackVector.clear();
            for (int i = 0; i < TN; i++) {
                VectorF m = delVector.multiplied((float)((TN - i) * fraction));
                stackVector.push(currentVector.added(m));
            }
//            currentVector = stackVector.firstElement();
            lastCommandTime = runtime.milliseconds();
            driveDuration = maxDuration;
            speed = sp;
            minPower = minPow;
            maxPower = maxPow;
        }
    }

    //calculate angle
    private double calcAngle(float x , float y){
        return Math.atan2(y,x);
    }

    private double CalculateSpeed(float x, float y, double mP) {
        double length = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        double proportion = length / 800;
        if (proportion >= 1) {
            proportion = 1;
        }
        if (proportion * mP <= .4) {
            return (.4);
        }

        return proportion * mP;
    }

    ////////////////////////////////////////////////////
    // Main drive thread
    ////////////////////////////////////////////////////
    public volatile boolean running = true;
    public void run() {
        double prevStep = runtime.milliseconds();
        try {
            while (running) {
//                DarudeAutoNav.ADBLog("Running. Current Vec: " + currentVector.get(0) + ":" + currentVector.get(1));
                // Check if running longer than requested duration without new commands
                if(runtime.milliseconds() - lastCommandTime > driveDuration) {
                    // Stop
                    VecDrive(0,0,0,0,0,10000); // Stop for 10 secs, than we will stop for another 10 and so on until new command arrives
                    DarudeAutoNav.ADBLog("Running too long. Stop!");
                }
                sleep(5);
                if(runtime.milliseconds() - prevStep > TRAN_STEP_TIME) {
                    /// pop from stack amd set it to current, if stack is empty get goal
                    if (stackVector.empty()) {
                        currentVector = goalVector;
                    } else {
                        currentVector = stackVector.pop();
                        DarudeAutoNav.ADBLog("Updating direction. Current Vec: " + currentVector.get(0) + ":" + currentVector.get(1));
                    }
                    prevStep = runtime.milliseconds();
                }

//                double angle = calcAngle(currentVector.get(0), currentVector.get(1));
//                double power = CalculateSpeed(currentVector.get(0), currentVector.get(1), .7);

                double x = currentVector.get(0);
                double y = currentVector.get(1);

                double l = Math.sqrt(x*x+y*y);
                if(l < 10) {
                    x = 0;
                    y = 0;
                    l = 0;
                }
                else {
                    x /= l;
                    y /= l;
                }

                double pow = l/150.0 * speed;
                if(pow > 0.01) {
                    if(pow > maxPower) pow = maxPower;
                    else if (pow< minPower) pow = minPower;
                }

                setMoveAngle(x, y, pow);
            }
        } catch (InterruptedException e) {}
    }

    public void Stop() {
        running = false;
    }

    public void setMoveAngle(double x, double y, double power) {
//        DarudeAutoNav.ADBLog("Drive vector: x:" + x + " y: " + y + " power: " + power);
        // Rotate 90 degrees
        double Xr = 0.707 * x + 0.707 * y;
        double Yr = 0.707 * x - 0.707 * y;

//        angle = angle + Math.PI / 4;
        DecimalFormat df = new DecimalFormat("#.##");
        try {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, GYRO_DEVICE_TIMEOUT_MS)) {
                if (yawPIDResult.isOnTarget()) {
                    double flp = power * Xr;
                    double frp = power * Yr;
                    double blp = frp;
                    double brp = flp;
                    frontLeft.setPower(flp);
                    frontRight.setPower(frp);
                    backLeft.setPower(blp);
                    backRight.setPower(brp);
//                    DarudeAutoNav.ADBLog("Motor speed: fl,br:" + Xr + " fr,bl: " + Yr + " power: " + power);
                } else {
                    double output = yawPIDResult.getOutput();
                    double flp = power * Xr;
                    double frp = power * Yr;
                    double blp = frp;
                    double brp = flp;
                    frontLeft.setPower(flp + output);
                    frontRight.setPower(frp - output);
                    backLeft.setPower(blp + output);
                    backRight.setPower(brp - output);
//                    DarudeAutoNav.ADBLog("Motor speed: fl,br:" + Xr + " fr,bl: " + Yr + " power: " + power);
                }

            } else {
                    /* A timeout occurred */
                Log.d("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        } catch (Exception ex) {
        }
    }


    public void driveDirection(double angle, double power, int time)
    {
        if(time <= 100) { time = 101; }
        moveAngle(-angle, power / 3, 25);
        moveAngle(-angle, 2 * power / 3, 25);
        moveAngle(-angle, power, time - 100);
        moveAngle(-angle, 2 * power / 3, 25);
        moveAngle(-angle, power / 3, 25);
        brake();
    }

    public void brake()
    {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public void testFrontLeft (double p) {
        frontLeft.setPower(p);
    }
    public void testBackLeft (double p) {
        backLeft.setPower(p);
    }
    public void testFrontRight (double p) {
        frontRight.setPower(p);
    }
    public void testBackRight (double p) {
        backRight.setPower(p);
    }


    private double limit(double a) {
        return Math.min(Math.max(a, MIN_MOTOR_OUTPUT_VALUE), MAX_MOTOR_OUTPUT_VALUE);
    }

    public void ReverseDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    private void moveAngle(double angle, double power, int time)
    {
        angle = angle*Math.PI/180 + Math.PI/4;
        double endTime = time + runtime.milliseconds();
        DecimalFormat df = new DecimalFormat("#.##");
        try {
            while (runtime.milliseconds() < endTime) {
                if (yawPIDController.waitForNewUpdate(yawPIDResult, GYRO_DEVICE_TIMEOUT_MS)) {
                    if (yawPIDResult.isOnTarget()) {
                        double flp = power * Math.cos(angle);
                        double frp = power * Math.sin(angle);
                        double blp = frp;
                        double brp = flp;
                        frontLeft.setPower(flp);
                        frontRight.setPower(frp);
                        backLeft.setPower(blp);
                        backRight.setPower(brp);
//                        telemetry.addData("Corr", 0);
//                        telemetry.addData("flp", flp);
//                        telemetry.addData("frp", frp);
//                        telemetry.addData("blp", blp);
//                        telemetry.addData("brp", brp);
//                        telemetry.addData("encoderBL", backLeft.getCurrentPosition());
//                        telemetry.addData("encoderBR", backRight.getCurrentPosition());
                    } else {
                        double output = yawPIDResult.getOutput();
                        double flp = power * Math.cos(angle);
                        double frp = power * Math.sin(angle);
                        double blp = frp;
                        double brp = flp;
                        frontLeft.setPower(flp + output);
                        frontRight.setPower(frp - output);
                        backLeft.setPower(blp + output);
                        backRight.setPower(brp - output);
//                        telemetry.addData("Corr", output);
//                        telemetry.addData("flp", flp + output);
//                        telemetry.addData("frp", frp - output);
//                        telemetry.addData("blp", blp + output);
//                        telemetry.addData("brp", brp - output);
//
//                        telemetry.addData("encoderBL", backLeft.getCurrentPosition());
//                        telemetry.addData("encoderBR", backRight.getCurrentPosition());
                    }
//                    telemetry.addData("Finished", "the initialization");
//                    telemetry.addData("Yaw", df.format(navx_device.getYaw()));
                    //telemetry.update();

                } else {
			        /* A timeout occurred */
                    Log.w("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
                }


//                frontLeft.setPower((power * Math.cos(angle + (Math.PI / 4))));
//                frontRight.setPower((power * Math.sin(angle + (Math.PI / 4))));
//                backLeft.setPower((power * Math.sin(angle + (Math.PI / 4))));
//                backRight.setPower((power * Math.cos(angle + (Math.PI / 4))));
                //telemetry.update();
            }
        }
        catch (Exception ex)
        {}
        //sleep(time);
    }
}


