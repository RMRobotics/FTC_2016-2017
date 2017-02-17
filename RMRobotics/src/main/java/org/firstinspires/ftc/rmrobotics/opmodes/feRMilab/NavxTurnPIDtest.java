package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

import java.text.DecimalFormat;

@Autonomous(name = "Turn")
@Disabled
public class NavxTurnPIDtest extends FeRMiLinear {

    private final double TARGET_ANGLE_DEGREES = 90.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;


    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, Direction.BACKWARD);

        navXPIDController yawPIDController = new navXPIDController(navx, navXPIDController.navXTimestampedDataSource.YAW);

        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);
        yawPIDController.setContinuous(true);
        yawPIDController.setOutputRange(MIN_MOTOR_OUTPUT_VALUE, MAX_MOTOR_OUTPUT_VALUE);
        yawPIDController.setTolerance(navXPIDController.ToleranceType.ABSOLUTE, TOLERANCE_DEGREES);
        yawPIDController.setPID(YAW_PID_P, YAW_PID_I, YAW_PID_D);

        waitForStart();

        yawPIDController.enable(true);

        /* Wait for new Yaw PID output values, then update the motors
           with the new PID value with each new output value.
         */

//        final double TOTAL_RUN_TIME_SECONDS = 30.0;
        int DEVICE_TIMEOUT_MS = 500;
        navXPIDController.PIDResult yawPIDResult = new navXPIDController.PIDResult();

        DecimalFormat df = new DecimalFormat("#.##");

        while (opModeIsActive()) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                if (yawPIDResult.isOnTarget()) {
                    setZeroMode(DcMotor.ZeroPowerBehavior.FLOAT);
//                    telemetry.addData("PIDOutput", df.format(0.00));
//                    telemetry.addData("STUPID");
                } else {
                    double output = yawPIDResult.getOutput();
                    setDrive(output, -output);
                    telemetry.addData("PIDOutput", df.format(output) + ", " +
                            df.format(-output));
                    addTelemetry();
                }

//                telemetry.addData("Yaw", df.format(navx.getYaw()));
//                telemetry.addData("target", df.format(navx.getYaw()));
            }
            telemetry.update();
        }

        stop();
    }

}
