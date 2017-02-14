package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

/**
 * Created by RM Robotics on 2/2/2017.
 */
@Autonomous(name = "RED: Ramp")
public class Ramp extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER);

        //turn at an angle
        turnRobotCorner(45);

        //drive forward
        driveRobot(1200, -.4);

        //face ramp
        turnRobot(130);

        //drive onto ramp
        driveRobot(1500, -.3);

        while (opModeIsActive()){
            addTelemetry();
        }
    }

    private void driveRobot(int distance, double power){
        double  startPos = FL.getCurrentPosition();
        while (FL.getCurrentPosition() - startPos > (-1*distance) && opModeIsActive()){
            setDrive(power);
        }
        setDrive(0);
        sleep(100);
    }

    private void turnRobotCorner(int a){
        while (Math.abs(navx.getYaw() + a) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + a > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw()) < (a - 10)){
                setDrive(scale * 0.25, 0);
            } else {
                setDrive(scale * 0.07, 0);
            }
        }
        setDrive(0);
        sleep(100);
    }

    private void turnRobot(int a){
        while (Math.abs(navx.getYaw() + a) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + a > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw()) < (a - 10)) {
                setDrive(scale * 0.25, scale*-0.25);
            } else {
                setDrive(scale * 0.07, scale*-0.07);
            }
        }
        setDrive(0);
        sleep(100);
    }
}
