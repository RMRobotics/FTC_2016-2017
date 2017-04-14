package org.firstinspires.ftc.rmrobotics.histoy.opmodes.feRMilab;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.text.DecimalFormat;

/**
 * Created by Peter on 11/29/16.
 */

@Autonomous(name = "sensors")
@Disabled
public class sensorAutoTest extends OpMode {
    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;
    //four motors
    private final int NAVX_DIM_I2C_PORT = 0;
    //navx device interface module sensor port is 0
    private AHRS navx;
    //gyro sensor with roll, pitch, and yaw
    private navXPIDController yawPIDController;
    //controls the yaw (the twist about a vertical axis)
    private ElapsedTime runtime = new ElapsedTime();

    private final byte NAVX_DEVICE_UPDATE_RATE_HZ = 50;

    private final double TARGET_ANGLE_DEGREES = 0.0;
    private final double TOLERANCE_DEGREES = 1.5;
    private final double MIN_MOTOR_OUTPUT_VALUE = -0.3;
    private final double MAX_MOTOR_OUTPUT_VALUE = 0.3;
    private final double YAW_PID_P = 0.03; //proportional error
    private final double YAW_PID_I = 0.0; //integral error
    private final double YAW_PID_D = 0.0; //derivative error

    private boolean calibration_complete = false;
    private boolean lineSeen = false;

    navXPIDController.PIDResult yawPIDResult;
    DecimalFormat df;
    //subclass of NumberFormat that can parse and format decimal numbers
    byte[] colorLinecache;
    //cache stores color line data
    I2cDevice colorLine;
    //color line sensor
    I2cDeviceSynch colorLinereader;
    //used to access color line sensor data

    @Override
    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");
        FR.setDirection(DcMotor.Direction.REVERSE);
        BR.setDirection(DcMotor.Direction.REVERSE);
        //reverse right motors because of drive train alignment
        FL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        FR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        BR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //float: slow gradual stop, brake: abrupt stop
        FL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BL.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        BR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //all motor rotation is measured using encoders
        navx = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"),
                NAVX_DIM_I2C_PORT,
                AHRS.DeviceDataType.kProcessedData,
                NAVX_DEVICE_UPDATE_RATE_HZ);
        //gyro set-up
        yawPIDController = new navXPIDController( navx, navXPIDController.navXTimestampedDataSource.YAW);
        //setup yaw controller
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        //set target yaw angle
        yawPIDController.setContinuous(true);
        //continuously calculate error based on update rate?
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        //restricts the controller's range of motor change when adjusting motors
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        //sets yaw tolerance
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        //sets proportional, integral, and derivative yaw values
        yawPIDController.enable(true);
        //enables controller
        df = new DecimalFormat("#.##");
        //formats output into desired decimal format
        telemetry.addData("Status", "Initialized");
        colorLine = hardwareMap.i2cDevice.get("colorBack");
        //sets up color line sensor
        colorLinereader = new I2cDeviceSynchImpl(colorLine, I2cAddr.create8bit(0x50), false);
        //sets up sensor reader with address
        colorLinereader.engage();
        colorLinereader.write8(3,0);
        //turns on light
    }

    public void init_loop() {
        calibration_complete = !navx.isCalibrating();
        //boolean is calibration complete
        if ( calibration_complete ) {
            navx.zeroYaw();
            //set yaw to zero
        }
        telemetry.addData("navx calibrated: ", !navx.isCalibrating());
        //updates calibration status to telemetry
    }

    @Override
    public void start() {
        navx.zeroYaw();
        //sets yaw to zero
        yawPIDResult = new navXPIDController.PIDResult();
        //PID values added together
    }

    @Override
    public void loop() {
        telemetry.addData("Status", "Running: " + runtime.toString());
        //add telemetry stuff
        colorLinecache = colorLinereader.read(0x04, 1);
        //read color number 1
        telemetry.addData("1 #L", colorLinecache[0] & 0xFF);
        telemetry.addData("2 A", colorLinereader.getI2cAddress().get8Bit());

        if (!lineSeen) {
        //if line isn't seen
            if (colorLinecache[0] != 14) {
            //if color sensor value is not 14 (color of white)
                if (yawPIDController.isNewUpdateAvailable(yawPIDResult)) {
                //if a new yaw value is available
                    if (yawPIDResult.isOnTarget()) {
                        setDrive(-0.5, 0.05, 0.05, -0.5);
                        telemetry.addData("Motor Output", df.format(0.00));
                        //if yaw is on target, drive the robot normally
                    } else {
                        double output = yawPIDResult.getOutput();
                        setDrive(-0.5 + output, 0.05 - output, 0.05 + output, -0.5 - output);
                        telemetry.addData("Motor Output", df.format(output) + ", " +
                                df.format(-output));
                        //if yaw is not on target, drive robot with adjustment
                    }
                }
            } else {
            //if color sensor value is 14
                lineSeen = true;
            }
        } else {
        //stop robot when line is seen
            setDrive(0, 0, 0, 0);
        }
    }

    private void setDrive(double p1, double p2, double p3, double p4) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p3);
        BR.setPower(p4);
        //set motor powers
    }
}
