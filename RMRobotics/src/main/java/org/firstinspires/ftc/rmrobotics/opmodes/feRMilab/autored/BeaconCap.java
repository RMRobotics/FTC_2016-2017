package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

import static org.firstinspires.ftc.rmrobotics.util.Direction.BACKWARD;
import static org.firstinspires.ftc.rmrobotics.util.Direction.CENTER;
import static org.firstinspires.ftc.rmrobotics.util.Direction.LEFT;
import static org.firstinspires.ftc.rmrobotics.util.Drive.RANGE;
import static org.firstinspires.ftc.rmrobotics.util.Drive.TIME;

/**
 * Created by Simon on 1/6/16.
 */
// RED TEAM


@Autonomous(name = "RED: Beacon")
public class BeaconCap extends FeRMiLinear {

    @Override
    public void runOpMode() {
        super.initialize(Color.RED, DcMotor.RunMode.RUN_USING_ENCODER, BACKWARD);

        // turn towards first beacon
        turn(LEFT, -37, 0.4);

        // drive forward until center color sensor detects line
        double initPos = Math.abs(FL.getCurrentPosition());
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            if (FL.getCurrentPosition() + initPos < -3500) {
                scale = 1;
            } else if (FL.getCurrentPosition() + initPos > -2400) {
                scale = -1;
            }
            if (FL.getCurrentPosition() + initPos > -2400) {
                setDrive(scale * 0.5);
            } else {
                setDrive(scale * 0.1);
            }
        }
        setDrive(0);
        sleep(100);

        // drives backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.07);
        }
        setDrive(0);

        // turn left towards beacon
        turn(CENTER, -86, 0.2);

        // drive forward until close enough to beacon
        drive(RANGE, 17, 0.1); // TODO: check if robot still drives in wrong direction

        boolean detected = false;
        double initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 200 && opModeIsActive()) {
            // move beacon pusher arm to appropriate location
            if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {
                //left is red, right is blue
                swingArm.setPosition(1.0);
                detected = true;
            } else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {
                swingArm.setPosition(0);
                detected = true;
            }
        }

        // drive forward to hit beacon
        if (detected) {
            sleep(150);
            drive(TIME, 600, -0.15); // TODO: check if robot still drives in wrong direction
        }

        // back away from beacon
        drive(RANGE, 30, 0.2); // TODO: check if robot still drives in wrong direction
        swingArm.setPosition(0.5);

        initTime = runtime.milliseconds();
        while(runtime.milliseconds()-initTime < 4500 && opModeIsActive()) {
            flyL.setPower(1.0);
            flyR.setPower(1.0);
            if (runtime.milliseconds() - initTime > 1000) {
                index.setPosition(.5);
                belt.setPower(0.5);
            } else if (runtime.milliseconds() - initTime > 1500) {
                index.setPosition(.5);
                belt.setPower(0);
            } else if (runtime.milliseconds() - initTime > 3000) {
                index.setPosition(.5);
                belt.setPower(1);
            }
        }

        belt.setPower(0);
        index.setPosition(.1);
        flyL.setPower(0);
        flyR.setPower(0);

        sleep(200);

        // FIRST BEACON DONE

        // turn towards second line
        turn(CENTER, -12, 0.15); // original power value was 0.15

        // drive forward slightly to move center color sensor off the first line
        drive(TIME, 350, -0.6); // TODO: check if robot still drives in wrong direction

        // drive forward until center color sensor detects second line
        initPos = Math.abs(FL.getCurrentPosition());
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            if (FL.getCurrentPosition() + initPos < -3000) {
                scale = 1;
            } else if (FL.getCurrentPosition() + initPos > -1500) {
                scale = -1;
            }
            if (FL.getCurrentPosition() + initPos > -1200) {
                setDrive(scale * 0.5);
            } else if (FL.getCurrentPosition() + initPos > -1800) {
                setDrive(scale * 0.3);
            } else {
                setDrive(scale * 0.1);
            }
        }
        setDrive(0);
        sleep(100);

        //drive backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.04);
        }
        setDrive(0);

        //turn left towards beacon
        turn(CENTER, -86, 0.2);

        // drive forward until close enough to beacon
        drive(RANGE, 14, 0.1);

        detected = false;
        initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 200 && opModeIsActive()) {
            // move beacon pusher arm to appropriate location
            if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 3){
                //left is red, right is blue
                swingArm.setPosition(1.0);
                detected = true;
            } else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 10){
                swingArm.setPosition(0);
                detected = true;
            }
        }

        // drive forward to hit beacon
        if (detected) {
            sleep(150);
            drive(TIME, 600, -.15);
        }

        // back away from beacon
        drive(RANGE, 20, .1);
        swingArm.setPosition(0.5);

        //turn towards center goal
        turn(CENTER, -60, 0.4);

        //drive to knock off cap ball
        drive(TIME, 2000, 0.4);

        stop();
    }
}
