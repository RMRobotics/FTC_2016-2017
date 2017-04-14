package org.firstinspires.ftc.rmrobotics.util.autonav;

import android.util.Log;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.rmrobotics.opmodes.sandstoRM.BetterDarudeAutoNav;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;

import static java.lang.Thread.sleep;

/**
 * Created by Peter on 1/26/2017.
 */

public class Drive2 implements Runnable {

    //runtime calculations
    ElapsedTime runtime = new ElapsedTime();

    // Opmode
    private LinearOpMode opmode = null;

    //navx
    private final int NAVX_DIM_I2C_PORT = 0;
    private AHRS navx_device;
    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;
    Telemetry telemetry;

    //Navx PID Variables
    private ZPIDController yawPIDController;
    private ZPIDController.PIDResult yawPIDResult;
    private final double REQUIRED_VOLTAGE = 13.0;
    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1;
    private final double YAW_PID_P = 0.013;
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
    private VoltageSensor voltSensor;

    private int prevFLEnc;
    private int prevFREnc;
    private int prevBLEnc;
    private int prevBREnc;

    public Drive2(
            DcMotor fl,
            DcMotor fr,
            DcMotor bl,
            DcMotor br,
            AHRS nx,
            Telemetry tl,
            LinearOpMode om) {
        //init motors
        frontLeft = fl;
        frontRight = fr;
        backLeft = bl;
        backRight = br;
        navx_device = nx;
        telemetry = tl;
        opmode = om;

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

        voltSensor = (VoltageSensor) frontLeft.getController();

        prevFLEnc = frontLeft.getCurrentPosition();
        prevBLEnc = backLeft.getCurrentPosition();
        prevBREnc = backRight.getCurrentPosition();
        prevFREnc = frontRight.getCurrentPosition();

        boolean calibration_complete = false;
        while (!calibration_complete) {
            /* navX-Micro Calibration completes automatically ~15 seconds after it is
            powered on, as long as the device is still.  To handle the case where the
            navX-Micro has not been able to calibrate successfully, hold off using
            the navX-Micro Yaw value until calibration is complete.
             */
            calibration_complete = !navx_device.isCalibrating();
            if (!calibration_complete) {
                telemetry.addData("navX-Micro", "Startup Calibration in Progress");
                telemetry.update();
            }
        }
        navx_device.zeroYaw();

        //initalize a PID controller
        yawPIDController = new ZPIDController(navx_device,
                navXPIDController.navXTimestampedDataSource.YAW);

        /* Configure the PID controller */
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE / 2, MAX_MOTOR_OUTPUT_VALUE / 2);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setP(YAW_PID_P);
        yawPIDController.enable(true);
        yawPIDResult = new ZPIDController.PIDResult();

    }

    private volatile VectorF reqV = new VectorF(0, 0);
    private volatile VectorF powVn = new VectorF(0, 0);
    private volatile VectorF powV = new VectorF(0, 0);

    private VectorF delV = new VectorF(0, 0);

    private volatile boolean newReq = false;

    private volatile double lastCommandTime = 0;
    private volatile double driveDuration = 0;
    private volatile double speed = 0;

    public void DriveByEncoders(double heading, double power, int distance) {
        resetDistance();
        VecDrive(10, 0, power, 20000);
        yawPIDController.setSetpoint(heading);
        while (opmode.opModeIsActive()) {
            int d = getDistance();
            if (d > distance) break;
//            BetterDarudeAutoNav.ADBLog("Dist: " + d);
            opmode.sleep(5);
        }
    }

    public void DriveByEncodersStr(double heading, double x, double y, double power, int distance) {
        resetDistance();
        VecDrive(x, y, power, 20000);
        yawPIDController.setSetpoint(heading);
        while (opmode.opModeIsActive()) {
            if (getDistance() > distance) break;
            opmode.sleep(5);
        }
    }

    public void TurnToAngle(double heading) {
        yawPIDController.setSetpoint(heading);

        try {
            while (opmode.opModeIsActive()) {
//                BetterDarudeAutoNav.ADBLog("TurnToAngle: Yaw=" + navx_device.getYaw() + ", AV=" + yawPIDController.angular_velocity);
                if (Math.abs(navx_device.getYaw() - heading) < TOLERANCE_DEGREES
                        && Math.abs(yawPIDController.angular_velocity) < 0.01) break;
                sleep(20);
            }
        } catch (InterruptedException ex) {
            BetterDarudeAutoNav.ADBLog("Interrupted TurnToAngle");
        }
    }

    public void SetYawPID_P(double p) {
        yawPIDController.setP(p);
    }


    public void VecDrive(double x, double y, double sp, int maxDuration) {
        synchronized (reqV) {
            reqV.put(0, (float) x / 2);
            reqV.put(1, (float) y);
            if (reqV.magnitude() < 0.05) {
                reqV.put(0, (float) 0);
                reqV.put(1, (float) 0);
            } else {
                reqV.multiply(((float) sp) / reqV.magnitude());
            }
            lastCommandTime = runtime.milliseconds();
            driveDuration = maxDuration;
//            speed = sp;
            newReq = true;
        }
    }

    public double getVoltCoef() {
        double coef = 1;
        double volt = voltSensor.getVoltage();
        if (volt <= 9) {
            coef = 1.3;
        } else if (volt <= 10) {
            coef = 1.1;
        } else if (volt <= 11) {
            coef = 1.0;
        }

//        BetterDarudeAutoNav.ADBLog("Voltage: " + volt + ", coeff: " + coef);
        return coef;
    }

    public void VecDriveBalanced(double x, double y, double sp, int maxDuration) {
        synchronized (reqV) {
            sp *= getVoltCoef();
            reqV.put(0, (float) x);
            reqV.put(1, (float) y);
            if (reqV.magnitude() < 0.05) {
                reqV.put(0, (float) 0);
                reqV.put(1, (float) 0);
            } else {
                reqV.multiply(((float) sp) / reqV.magnitude());
                reqV.getData()[0] /= 2; // Adjust X power
            }
            lastCommandTime = runtime.milliseconds();
            driveDuration = maxDuration;
//            speed = sp;
            newReq = true;
        }
    }

    ////////////////////////////////////////////////////
    // Main drive thread
    ////////////////////////////////////////////////////
    public volatile boolean running = true;

    public void run() {
        //for boost, unused
        float c0 = (float) .5;
        float c1 = (float) 1;
        float c2 = (float) 0;
//        double prevStep = runtime.milliseconds();


//        try {
        while (running && opmode.opModeIsActive()) {

            if (newReq) {
                synchronized (reqV) {
                    newReq = false;
                }
            }
            powV = reqV;

            if (runtime.milliseconds() - lastCommandTime > driveDuration) {
                // Stop
                synchronized (reqV) {
                    reqV.put(0, 0);
                    reqV.put(1, 0);
                    newReq = true;
                }
//                    BetterDarudeAutoNav.ADBLog("Running too long. Stop!");
            }

//                BetterDarudeAutoNav.ADBLog("Running. Current pow: " + powV.get(0) + ":" + powV.get(1));
//                BetterDarudeAutoNav.ADBLog("Running. Current req: " + reqV.get(0) + ":" + reqV.get(1));
//                BetterDarudeAutoNav.ADBLog("Running. Current del: " + delV.get(0) + ":" + delV.get(1));
            setMoveAngle(powV.get(0), powV.get(1), 1);
        }

//        catch (InterruptedException e) {
//            brake();
//            emergencyBrake();
//        }
        brake();
        emergencyBrake();
    }

    public void Stop() {
        brake();
        running = false;
        emergencyBrake();
    }


    private int prevLF = 0;
    private int prevLB = 0;
    private int prevRF = 0;
    private int prevRB = 0;
    private boolean enable_strife_counter = false;
    private float deltaY = 0;

    public void resetYDist() {
        prevLF = frontLeft.getCurrentPosition();
        prevLB = backLeft.getCurrentPosition();
        prevRF = frontRight.getCurrentPosition();
        prevRB = backRight.getCurrentPosition();
    }

    public double getYDistIncr() {

        int lf = frontLeft.getCurrentPosition();
        int lb = backLeft.getCurrentPosition();
        int left = (lf - prevLF) - (lb - prevLB);
        int rf = frontRight.getCurrentPosition();
        int rb = backRight.getCurrentPosition();
        int right = (rb - prevRB) - (rf - prevRF);

//        BetterDarudeAutoNav.ADBLog("lf: " + (lf-prevLF) + ", lb: " + (lb-prevLB) + ", rb: " + (rb-prevRB) + ", rf: " + (rf-prevRF));

        prevLF = lf;
        prevLB = lb;
        prevRF = rf;
        prevRB = rb;

        double res = (left + right);
        if (res < 0) return res / 16;
        else return res / 12;

    }

    public void setMoveAngle(double x, double y, double power) {
//        BetterDarudeAutoNav.ADBLog("Power: " + x + ", " + y);
        // Rotate 90 degrees
        double Xr = 0.707 * x + 0.707 * y;
        Xr *= power;
        double Yr = 0.707 * x - 0.707 * y;
        Yr *= power;
        boolean moving = (Math.abs(Xr) + Math.abs(Yr)) > 0.001;

        try {
            yawPIDController.moving = moving;
            if (yawPIDController.waitForNewUpdate(yawPIDResult, GYRO_DEVICE_TIMEOUT_MS)) {
                double av = yawPIDResult.angular_velocity;
                double error = yawPIDResult.error;
                if (yawPIDResult.isOnTarget()) {
                    double flp = Xr;
                    double frp = Yr;
                    double blp = frp;
                    double brp = flp;
                    frontLeft.setPower(flp);
                    frontRight.setPower(frp);
                    backLeft.setPower(blp);
                    backRight.setPower(brp);
//                    BetterDarudeAutoNav.ADBLog("On target motor speed: fl,br:" + flp + ", fr,bl: " + frp
//                            + ", err: " + error + ", av= " + av);
                } else {
                    double output = yawPIDResult.getOutput();
                    double flp = Xr;
                    double frp = Yr;
                    double blp = frp;
                    double brp = flp;

                    if (!moving) {
                        output = yawPIDResult.getStationaryOutput(0.1);
                    }

                    frontLeft.setPower(flp + output);
                    frontRight.setPower(frp - output);
                    backLeft.setPower(blp + output);
                    backRight.setPower(brp - output);
//                    BetterDarudeAutoNav.ADBLog("Motor speed: fl,br:" + flp + ", fr,bl: " + frp
//                            + ", err: " + error + ",out: " + output + ", av= " + av);
                }
                // Calculate odometer
//                double a = Math.toRadians(yawPIDController.prev_process_value);
//                int new_LCount = frontLeft.getCurrentPosition();
//                int new_RCount = frontRight.getCurrentPosition();
//                int average = ((prevLCount - new_LCount) + (prevRCount - new_RCount))/2;
//                deltaX += Math.cos(a)*average;
//                deltaY += Math.sin(a)*average;
//                prevLCount = new_LCount;
//                prevRCount = new_RCount;
//                BetterDarudeAutoNav.ADBLog("Odometer X: " + deltaX + ", Y:" + deltaY);
            } else {
                    /* A timeout occurred */
                Log.d("navXDriveStraightOp", "Yaw PID waitForNewUpdate() TIMEOUT.");
            }
        } catch (Exception ex) {
        }
    }

    public void brake() {
        synchronized (reqV) {
            delV.put(0, 0);
            delV.put(1, 0);
            reqV.put(0, 0);
            reqV.put(1, 0);
            powV.put(0, 0);
            powV.put(1, 0);
            lastCommandTime = runtime.milliseconds();
            driveDuration = 30000;
        }
    }

    public void brake_and_wait() {
        try {
            while (opmode.opModeIsActive()) {
                synchronized (reqV) {
                    delV.put(0, 0);
                    delV.put(1, 0);
                    reqV.put(0, 0);
                    reqV.put(1, 0);
                    powV.put(0, 0);
                    powV.put(1, 0);
                    lastCommandTime = runtime.milliseconds();
                    driveDuration = 30000;
                }
                if (!navx_device.isMoving()) break;
//                BetterDarudeAutoNav.ADBLog("Waiting to stop");
                sleep(30);
            }
        } catch (InterruptedException ex) {
        }
    }


    public void emergencyBrake() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    public int getDistance() {
        int c = frontLeft.getCurrentPosition();
        int r = Math.abs(c - prevFLEnc);
        c = backLeft.getCurrentPosition();
        r += Math.abs(c - prevBLEnc);
        c = frontRight.getCurrentPosition();
        r += Math.abs(c - prevFREnc);
        c = backRight.getCurrentPosition();
        r += Math.abs(c - prevBREnc);

//        BetterDarudeAutoNav.ADBLog("Distance: " + frontLeft.getCurrentPosition() + ", " + frontRight.getCurrentPosition() + ", " +
//                backLeft.getCurrentPosition() + ", " + backRight.getCurrentPosition() + ", r:" + r);

        return r * 16 / 100;
    }

    public void resetDistance() {
        int c = frontLeft.getCurrentPosition();
        prevFLEnc = c;
        c = backLeft.getCurrentPosition();
        prevBLEnc = c;
        c = frontRight.getCurrentPosition();
        prevFREnc = c;
        c = backRight.getCurrentPosition();
        prevBREnc = c;
    }

    public int testFrontLeft(double p) {
        frontLeft.setPower(p);
        return frontLeft.getCurrentPosition();
    }

    public int testBackLeft(double p) {
        backLeft.setPower(p);
        return backLeft.getCurrentPosition();
    }

    public int testFrontRight(double p) {
        frontRight.setPower(p);
        return frontRight.getCurrentPosition();
    }

    public int testBackRight(double p) {
        backRight.setPower(p);
        return backRight.getCurrentPosition();
    }

    public void ReverseDirection() {
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);
    }
}


