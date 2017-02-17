package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by RM Robotics on 2/4/2017.
 */
@Autonomous(name = "RED: Cap")
public class Cap extends FeRMiLinear {

    @Override
    public void runOpMode() throws InterruptedException {
        super.initialize(Color.NEITHER, DcMotor.RunMode.RUN_USING_ENCODER, Direction.FORWARD);

        //turn at an angle
        turnRobotCorner(45);

        //drive forward
        driveRobot(3000, -.3);

        //wait
        sleep(3000);

        //drive forward
        driveRobot(500, -.2);

        //knock-off capball
        //turnRobot(500);

        while (opModeIsActive()){
            addTelemetry();
        }
    }

    private void driveRobot(int distance, double power){
        double startPos = FL.getCurrentPosition();
        while (FL.getCurrentPosition() - startPos > (-1*distance) && opModeIsActive()){
            //while robot has not traveled distance amount
            setDrive(power);
        }
        setDrive(0);
        sleep(100);
    }

    private void turnRobotCorner(int a){
        while (Math.abs(navx.getYaw() + a) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + a > 0) {
                //if robot has turned less than a degrees in left direction
                scale = -1;
            } else {
                //if robot has turned more than a degrees in left direction
                scale = 1;
            }
            if (Math.abs(navx.getYaw()) < (a - 10)){
                //if robot has turned less than (a-10) degrees in either direction
                //then turns robot at a faster speed
                setDrive(scale * 0.25, 0);
            } else {
                //if robot has turned more than (a-10) degrees in either direction
                //turns robot at a slower speed
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
                //if robot has turned less than a degrees in left direction
                scale = -1;
            } else {
                //if robot has turned more than a degrees in left direction
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
