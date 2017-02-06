package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

/**
 * Created by Simon on 1/6/16.
 */
// RED TEAM


@Autonomous(name = "BeaconCapTest")
public class BeaconCapTest extends FeRMiLinear {

    @Override
    public void runOpMode() {
        // initialize
        super.initialize(Color.RED);

        // turn towards first beacon
        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (Math.abs(navx.getYaw() + 37) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 37 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 37) > 15) {
                setDrive(scale * 0.4, 0);
            } else {
                setDrive(scale * 0.07, 0);
            }
        }
        addTelemetry();
//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // drive forward until center color sensor detects line
        double initPos = Math.abs(FL.getCurrentPosition());
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            if (FL.getCurrentPosition() - initPos < -3500) {
                scale = 1;
            } else if (FL.getCurrentPosition() - initPos > -2400) {
                scale = -1;
            }
            if (FL.getCurrentPosition() - initPos > -2400) {
                setDrive(scale * 0.5);
            } else {
                setDrive(scale * 0.1);
            }
        }
        addTelemetry();
        setDrive(0, 0, 0, 0);
        sleep(100);

        // drives backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.07);
        }
        addTelemetry();
        setDrive(0, 0, 0, 0);
        sleep(100);

        // turn left until back color sensor also detects the line
        while (Math.abs(navx.getYaw() + 86) > 5 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 86 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 86) > 15) {
                setDrive(scale * 0.15, -scale * 0.15);
            } else {
                setDrive(scale * 0.07, -scale * 0.07);
            }
        }
        addTelemetry();
        setDrive(0, 0, 0, 0);
        sleep(100);

        // turn right to correct overturning
        while (navx.getYaw() > -85 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 85 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            setDrive(scale * 0.05, -scale * 0.05);
        }
        addTelemetry();
        setDrive(0);
        sleep(100);

        // drive forward until close enough to beacon
        while (rangeReader.read(0x04, 2)[0] > 17 && opModeIsActive()) {
            setDrive(-0.1);
            addTelemetry();
        }
        setDrive(0);
        setDrive(0, 0, 0, 0);
        sleep(100);

        // move beacon pusher arm to appropriate location
        if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.65);
        }
        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.4);
        }
        addTelemetry();


        double initTime = runtime.milliseconds();
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 13 && opModeIsActive()) {
        while (runtime.milliseconds() - initTime < 1000 && opModeIsActive()) {
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 12 && opModeIsActive()) {
            setDrive(-0.15, -0.15, -0.15, -0.15);
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // back away from beacon
        while (rangeReader.read(0x04, 2)[0] < 17 && opModeIsActive()) {
            setDrive(0.2, 0.2, 0.2, 0.2);
            addTelemetry();
        }
        swingArm.setPosition(0.5);


        //FIRST BEACON DONE


//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // turn towards second line
        while (navx.getYaw() < -10 && opModeIsActive()) {
            if (Math.abs(navx.getYaw()) > 25) {
                setDrive(.1, -0.1, .1, -0.1);
            } else {
                setDrive(0, -0.07, 0, -0.07);
            }
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        int curEnc = FL.getCurrentPosition();

        // drive forward slightly to move center color sensor off the first line
        double start = runtime.milliseconds();
        while (runtime.milliseconds() - start < 1000 && opModeIsActive()) {
            setDrive(-0.6);
            addTelemetry();
        }

        // drive forward until center color sensor detects second line
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            int relDis = FL.getCurrentPosition() - curEnc;
            if (relDis > -1500) {
                setDrive(-0.35);
            } else {
                if (relDis > -1000) {
                    scale = -1;
                } else if(relDis < -3000) {
                    scale = 1;
                }
                setDrive(scale*0.07);
            }
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        setDrive(0);
        sleep(100);

        //drive backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.04);
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

        //turn left towards beacon
        while (Math.abs(navx.getYaw() + 88) > 3 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 90 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() + 88) > 15) {
                setDrive(scale * 0.2, -scale * 0.2);
            } else {
                setDrive(scale * 0.07, -scale * 0.07);
            }
            //addTelemetry();
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

        //turn right to correct for overturning
        while (Math.abs(navx.getYaw() + 88) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() + 88 > 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            setDrive(0, -scale * 0.07);
        }
        addTelemetry();

        setDrive(0);
        sleep(100);

        // drive forward until close enough to beacon
        while (rangeReader.read(0x04, 2)[0] > 15 && opModeIsActive()) {
            setDrive(-0.1);
            addTelemetry();
        }

        setDrive(0);
        sleep(100);

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // move beacon pusher arm to appropriate location
        if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.68);
        }
        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.20);
        }
        addTelemetry();

        initTime = runtime.milliseconds();
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 13 && opModeIsActive()) {
        while (runtime.milliseconds() - initTime < 1000 && opModeIsActive()) {
        // drive forward to hit beacon
        //while (rangeReader.read(0x04, 2)[1] < 12 && opModeIsActive()) {
            setDrive(-0.15, -0.15, -0.15, -0.15);
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // back away from beacon
        while (rangeReader.read(0x04, 2)[0] < 20 && opModeIsActive()) {
            setDrive(0.1, 0.1, 0.1, 0.1);
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);
        sleep(100);

        //turn towards center goal
        while (Math.abs(navx.getYaw() + 45) > 3 && opModeIsActive()){
            if (Math.abs(navx.getYaw()) > 45) {
                setDrive(0.4, 0, 0.4, 0);
            } else {
                setDrive(0.1, 0, 0.1, 0);
            }
            addTelemetry();
        }

        setDrive(0, 0, 0, 0);
        sleep(100);

//        double currentFLencoder = FL.getCurrentPosition();
//        while (FL.getCurrentPosition() - currentFLencoder < -600){
//            setDrive(.4, .4, .4, .4);
//        }
//
//        setDrive(0, 0, 0, 0);
//        sleep(100);

        stop();
    }

}