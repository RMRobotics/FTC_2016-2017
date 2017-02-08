package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab;

import com.kauailabs.navx.ftc.navXPIDController;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.rmrobotics.util.Color;

import java.text.DecimalFormat;

@Autonomous(name = "PIDAuto")
public class PIDAuto extends FeRMiLinear {

    private double TARGET_ANGLE_DEGREES = -55.0;
    private final double TOLERANCE_DEGREES = 2.0;
    private final double MIN_MOTOR_OUTPUT_VALUE = -1.0;
    private final double MAX_MOTOR_OUTPUT_VALUE = 1.0;
    private final double YAW_PID_P = 0.005;
    private final double YAW_PID_I = 0.0;
    private final double YAW_PID_D = 0.0;

    private double MIN_POWER = .1;
    private double MAX_POWER = .9;
    private boolean IS_DONE = false;


    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER);

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

        double distance = -1000;
        int distanceInt = -1000;
        setEnc(distanceInt);
        while (FL.getCurrentPosition() > distance){
            FL.setPower(Range.clip((-FL.getCurrentPosition()+FL.getTargetPosition())/distance,-7,-.07));
            FR.setPower(Range.clip((-FR.getCurrentPosition()+FR.getTargetPosition())/distance,-.7,-.07));
            BL.setPower(Range.clip((-BL.getCurrentPosition()+BL.getTargetPosition())/distance,-.7,-.07));
            BR.setPower(Range.clip((-BR.getCurrentPosition()+BR.getTargetPosition())/distance,-.7,-.07));
        }

        while (!IS_DONE) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                if (yawPIDResult.isOnTarget()||navx.getYaw()<TARGET_ANGLE_DEGREES) {
                    setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
                    setDrive(0);
                    IS_DONE = true;
//                    telemetry.addData("PIDOutput", df.format(0.00));
//                    telemetry.addData("STUPID");
                } else {
                    double output = yawPIDResult.getOutput();
                    if (output > MAX_POWER){
                        output = MAX_POWER;
                    }else if (output < MIN_POWER){
                        output = MIN_POWER;
                    }
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
        IS_DONE = false;

        distance = -4000;

        distanceInt = -4000;
        setEnc(distanceInt);

        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            FL.setPower(Range.clip((-FL.getCurrentPosition()+FL.getTargetPosition())/distance,-.5,-.07));
            FR.setPower(Range.clip((-FR.getCurrentPosition()+FR.getTargetPosition())/distance,-.5,-.07));
            BL.setPower(Range.clip((-BL.getCurrentPosition()+BL.getTargetPosition())/distance,-.5,-.07));
            BR.setPower(Range.clip((-BR.getCurrentPosition()+BR.getTargetPosition())/distance,-.5,-.07));
        }

        TARGET_ANGLE_DEGREES = 40.0;
        yawPIDController.setSetpoint(TARGET_ANGLE_DEGREES);

        while (colorBackReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            if (yawPIDController.waitForNewUpdate(yawPIDResult, DEVICE_TIMEOUT_MS)) {
                if (yawPIDResult.isOnTarget()) {
                    setZeroMode(DcMotor.ZeroPowerBehavior.BRAKE);
                    setDrive(0);
                    IS_DONE = true;
//                    telemetry.addData("PIDOutput", df.format(0.00));
//                    telemetry.addData("STUPID");
                } else {
                    double output = yawPIDResult.getOutput();
                    if (output > MAX_POWER){
                        output = MAX_POWER;
                    }else if (output < MIN_POWER){
                        output = MIN_POWER;
                    }
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
    }

}
