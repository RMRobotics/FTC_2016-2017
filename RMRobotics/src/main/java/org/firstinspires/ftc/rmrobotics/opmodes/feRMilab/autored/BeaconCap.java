package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

import static org.firstinspires.ftc.rmrobotics.util.Direction.BACKWARD;

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
        turnCorner(37, 0.4, ); // changed the angle of turn

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
        turnCorner(86, 0.2, ); //changed beacon to
        /*
        while (Math.abs(navx.getYaw() + 86) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 90 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 88) > 10) {
                setDrive(scale * 0.2, -scale * 0.2);
            } else {
                setDrive(scale * 0.07, -scale * 0.07);
            }
        }
        */

        // drive forward until close enough to beacon
        driveToRange(-0.1, 17);
        /*while (rangeReader.read(0x04, 2)[0] > 17 && opModeIsActive()) {
            setDrive(-0.1);
        }
        setDrive(0);*/

        boolean detected = false;
        double initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 200 && opModeIsActive()) {
            // move beacon pusher arm to appropriate location
            if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {
                //left is red, right is blue
                swingArm.setPosition(0.65);
                detected = true;
            } else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {
                swingArm.setPosition(0.25);
                detected = true;
            }
        }

        // drive forward to hit beacon
        driveTime(-.15, 600);
        /*initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 600 && opModeIsActive() && detected) {
            setDrive(-0.15);
        }*/

        // back away from beacon
        driveAwayRange(0.2, 15);
        /*while (rangeReader.read(0x04, 2)[0] < 15 && opModeIsActive()) {
            setDrive(0.2);
        }
        swingArm.setPosition(0.4);

        setDrive(0);*/

        initTime = runtime.milliseconds();
        while(runtime.milliseconds()-initTime < 3500 && opModeIsActive()) {
            flyL.setPower(0.95);
            flyR.setPower(0.95);
            if (runtime.milliseconds() - initTime > 1000) {
                index.setPosition(.5);
                belt.setPower(1);
            } else if (runtime.milliseconds() - initTime > 1500) {
                index.setPosition(.5);
                belt.setPower(0);
            } else if (runtime.milliseconds() - initTime > 2500) {
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
        turnCenter(2, .15);
        /*while (navx.getYaw() < -2 && opModeIsActive()) {
            if (Math.abs(navx.getYaw()) > 25) {
                setDrive(.15, -0.15);
            } else {
                setDrive(0.07, -0.07);
            }
        }
        setDrive(0)*/

        // drive forward slightly to move center color sensor off the first line
        driveTime(-0.6, 450);
        /*double start = runtime.milliseconds();
        while (runtime.milliseconds() - start < 450 && opModeIsActive()) {
            setDrive(-0.6);
        }*/

        // drive forward until center color sensor detects second line
        double checkFirst = -2500;
        initPos = Math.abs(FL.getCurrentPosition());
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            if (FL.getCurrentPosition() + initPos < -3000) {
                scale = 1;
            } else if (FL.getCurrentPosition() + initPos > -1500) {
                scale = -1;
            }
            if (FL.getCurrentPosition() + initPos > -1500) {
                setDrive(scale * 0.5);
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

        //turn left towards beacon
        turnCenter(-86, 0.2);
        /*while (Math.abs(navx.getYaw() + 86) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 90 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 88) > 10) {
                setDrive(scale * 0.2, -scale * 0.2);
            } else {
                setDrive(scale * 0.07, -scale * 0.07);
            }
        }
        setDrive(0);*/

        // drive forward until close enough to beacon
        driveToRange(-0.1, 14);
        /*while (rangeReader.read(0x04, 2)[0] > 14 && opModeIsActive()) {
            setDrive(-0.1);
        }
        setDrive(0);
        sleep(100);*/

        detected = false;
        initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 200 && opModeIsActive()) {
            // move beacon pusher arm to appropriate location
            if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 3){
                //left is red, right is blue
                swingArm.setPosition(0.65);
                detected = true;
            } else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 10){
                swingArm.setPosition(0.25);
                detected = true;
            }
        }

        // drive forward to hit beacon
        driveTime(-.15, 600);
        /*initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 600 && opModeIsActive() && detected) {
            setDrive(-0.15);
        }*/

        // back away from beacon
        driveAwayRange(.1, 20);
        /*while (rangeReader.read(0x04, 2)[0] < 20 && opModeIsActive() && detected) {
            setDrive(0.1);
        }*/

        //turn towards center goal
        turnCenter(45, 0.4);

        //drive to knock off cap ball
        driveTime(.4, 2000);
        /*initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 2000 && opModeIsActive()) {
            setDrive(0.4);
        }

        setDrive(0);*/

        stop();
    }
}