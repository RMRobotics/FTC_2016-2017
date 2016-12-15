package org.firstinspires.ftc.rmrobotics.opmodes.theRMite;

import android.graphics.Path;

import com.kauailabs.navx.ftc.AHRS;
import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Simon on 12/15/16.
 */

@Autonomous(name = "navx", group = "sensors")
public class NavX extends OpMode {

    DcMotor FL;
    DcMotor FR;
    DcMotor BL;
    DcMotor BR;

    private final int NAVX_DIM_I2C_PORT = 0; //TODO: change this to match the port on the Device Interface Module
    private AHRS navx;
    private navXPIDController yawPIDController;
    private final byte NAVX_UPDATE_RATE_HZ = 50;

    private final double TOLERANCE_DEGREES = 1.5;
    private final double MIN_MOTOR_OUTPUT_VALUE = -0.2;
    private final double MAX_MOTOR_OUTPUT_VALUE = 0.2;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;
    navXPIDController.PIDResult yawPIDResult;

    @Override
    public void init() {
        FL = hardwareMap.dcMotor.get("FL");
        FR = hardwareMap.dcMotor.get("FR");
        BL = hardwareMap.dcMotor.get("BL");
        BR = hardwareMap.dcMotor.get("BR");

        navx = AHRS.getInstance(hardwareMap.deviceInterfaceModule.get("dim"), NAVX_DIM_I2C_PORT, AHRS.DeviceDataType.kProcessedData, NAVX_UPDATE_RATE_HZ);
        yawPIDController = new navXPIDController(navx, navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);
        yawPIDController.enable(true);
    }

    @Override
    public void loop() {
        yawPIDController.setSetpoint(90); //TODO: test changes to this value
        while (!yawPIDController.isOnTarget()) {
            double output = yawPIDResult.getOutput();
            setDrive(output, -output);
        }
    }

    private void setDrive(double p1, double p2) {
        FL.setPower(p1);
        FR.setPower(p2);
        BL.setPower(p1);
        BR.setPower(p2);
    }
}
