package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;
import org.firstinspires.ftc.rmrobotics.util.Direction;

/**
 * Created by RM Robotics on 2/11/2017.
 */
@Autonomous(name = "BeaconCapBlue")
public class BlueBeaconCap extends FeRMiLinear {
    @Override
    public void runOpMode() {
        super.initialize(Color.BLUE, DcMotor.RunMode.RUN_USING_ENCODER, Direction.BACKWARD);

        // turn towards first beacon
        turnRobotCorner(37);
//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // drive forward until center color sensor detects line
        double initPos = Math.abs(FR.getCurrentPosition());
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            int scale = -1;
            if (FR.getCurrentPosition() + initPos < -3500) {
                scale = 1;
            } else if (FR.getCurrentPosition() + initPos > -2400) {
                scale = -1;
            }
            if (FR.getCurrentPosition() + initPos > -2400) {
                setDrive(scale * 0.5);
            } else {
                setDrive(scale * 0.1);
            }
        }
        setDrive(0);
        sleep(100);
        addTelemetry();

        // drives backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.07);
        }
        addTelemetry();
        setDrive(0);
        sleep(100);

        // turn left until back color sensor also detects the line
        while (Math.abs(navx.getYaw() - 86) > 5 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() - 86 < 0) {
                scale = -1;
            } else {
                scale = 1;
            }
            if (Math.abs(navx.getYaw() - 86) > 15) {
                setDrive(-scale * 0.25, scale * 0.25);
            } else {
                setDrive(-scale * 0.07, scale * 0.07);
            }
        }
        setDrive(0, 0, 0, 0);
        sleep(100);
        addTelemetry();

        // turn right to correct overturning
        while (navx.getYaw() < 85 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() - 85 < 0) {
                scale = 1;
            } else {
                scale = -1;
            }
            setDrive(-scale * 0.05, scale * 0.05);
        }
        setDrive(0);
        sleep(100);
        addTelemetry();

        // drive forward until close enough to beacon
        while (rangeReader.read(0x04, 2)[0] > 17 && opModeIsActive()) {
            setDrive(0.1);
            addTelemetry();
        }
        setDrive(0);
        setDrive(0, 0, 0, 0);
        sleep(100);

////        setDrive(0, 0, 0, 0);
////        sleep(100);
////
////        // give beacon pusher enough time to detect color of beacon
//        double startTime = runtime.milliseconds();
////        while (runtime.milliseconds() - startTime < 500) {
////            addTelemetry();
////        }
//
//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // move beacon pusher arm to appropriate location
        if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 3){
            //left is red, right is blue
            swingArm.setPosition(.85);
        }
        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// || Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 10){
            swingArm.setPosition(.2);
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
        while (rangeReader.read(0x04, 2)[0] < 25 && opModeIsActive()) {
            setDrive(0.2, 0.2, 0.2, 0.2);
            addTelemetry();
        }
        swingArm.setPosition(0.5);

        setDrive(0);


        initTime = runtime.milliseconds();
        while(runtime.milliseconds()-initTime < 3500 && opModeIsActive()){
            flyL.setPower(0.985);
            flyR.setPower(0.985);

            if(runtime.milliseconds()-initTime > 1000){
                index.setPosition(.5);
                belt.setPower(1);
            }else if (runtime.milliseconds()-initTime > 1500) {
                index.setPosition(.5);
                belt.setPower(0);
            }else if (runtime.milliseconds()-initTime > 2500) {
                index.setPosition(.5);
                belt.setPower(1);
            }

//            if(runtime.milliseconds()-initTime > 2100) {
//                belt.setPower(1.0);
//            }else if(runtime.milliseconds()-initTime > 2400){
//                belt.setPower(0);
//            }
        }

        belt.setPower(0);
        index.setPosition(.1);
        flyL.setPower(0);
        flyR.setPower(0);

        //FIRST BEACON DONE


//        setDrive(0, 0, 0, 0);
//        sleep(100);

        // turn towards second line
        while (navx.getYaw() > 10 && opModeIsActive()) {
            if (Math.abs(navx.getYaw()) > 25) {
                setDrive(-.15, 0.15, -.15, 0.15);
            } else {
                setDrive(0, 0.07, 0, 0.07);
            }
            addTelemetry();
        }

//        setDrive(0, 0, 0, 0);
//        sleep(100);

        while (opModeIsActive()) {
            addTelemetry();
            setZeroMode(DcMotor.ZeroPowerBehavior.FLOAT);
            setDrive(0);
        }

//        int curEnc = FL.getCurrentPosition();
//
//        // drive forward slightly to move center color sensor off the first line
//        double start = runtime.milliseconds();
//        while (runtime.milliseconds() - start < 1000 && opModeIsActive()) {
//            setDrive(-0.6);
//            addTelemetry();
//        }
//
//        // drive forward until center color sensor detects second line
//        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
//            int scale = -1;
//            int relDis = FL.getCurrentPosition() - curEnc;
//            if (relDis > -3000) {
//                setDrive(-0.35);
//            } else {
//                if (relDis > -3500) {
//                    scale = -1;
//                } else if(relDis < -3500) {
//                    scale = 1;
//                }
//                setDrive(scale*0.07);
//            }
//        }
//
////        setDrive(0, 0, 0, 0);
////        sleep(100);
//
//        setDrive(0);
//        sleep(100);
//
//        //drive backwards to correct for overshooting
//        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
//            setDrive(0.04);
//        }
//        addTelemetry();
//
//        setDrive(0);
//        sleep(100);
//
//        //turn left towards beacon
//        while (Math.abs(navx.getYaw() + 88) > 3 && opModeIsActive()) {
//            int scale;
//            if (navx.getYaw() + 90 > 0) {
//                scale = -1;
//            } else {
//                scale = 1;
//            }
//            if (Math.abs(navx.getYaw() + 88) > 15) {
//                setDrive(scale * 0.2, -scale * 0.2);
//            } else {
//                setDrive(scale * 0.07, -scale * 0.07);
//            }
//            //addTelemetry();
//        }
//        addTelemetry();
//
//        setDrive(0);
//        sleep(100);
//
//        //turn right to correct for overturning
//        while (Math.abs(navx.getYaw() + 88) > 2 && opModeIsActive()) {
//            int scale;
//            if (navx.getYaw() + 88 > 0) {
//                scale = -1;
//            } else {
//                scale = 1;
//            }
//            setDrive(0, -scale * 0.07);
//        }
//        addTelemetry();
//
//        setDrive(0);
//        sleep(100);
//
////        setDrive(0, 0, 0, 0);
////        sleep(100);
//
////        // turn until back color sensor also detects the line
////        while (colorBackReader.read(0x08, 1)[0] < 25) {
//////            if (FL.getCurrentPosition() > -7000) { //test encoder value
//////                setDrive(-0.1, 0.1, -0.1, 0.1);
//////            } else {
////            setDrive(-0.06, 0.06, -0.06, 0.06);
//////            }
////        }
//
////        setDrive(0, 0, 0, 0);
////        sleep(100);
//
//        // drive forward until close enough to beacon
//        while (rangeReader.read(0x04, 2)[0] > 15 && opModeIsActive()) {
//            setDrive(-0.1);
//            addTelemetry();
//        }
//
//        setDrive(0);
//        sleep(100);
//
////        setDrive(0, 0, 0, 0);
////        sleep(100);
//
//        // move beacon pusher arm to appropriate location
//        if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {// && colorRightCache[0] == 3){
//            //left is red, right is blue
//            swingArm.setPosition(.85);
//        }
//        else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {// && colorRightCache[0] == 10){
//            swingArm.setPosition(.20);
//        }
//        addTelemetry();
//
//        initTime = runtime.milliseconds();
//        // drive forward to hit beacon
//        //while (rangeReader.read(0x04, 2)[1] < 13 && opModeIsActive()) {
//        while (runtime.milliseconds() - initTime < 1000 && opModeIsActive()) {
//        // drive forward to hit beacon
//        //while (rangeReader.read(0x04, 2)[1] < 12 && opModeIsActive()) {
//            setDrive(-0.15, -0.15, -0.15, -0.15);
//            addTelemetry();
//        }
//
////        setDrive(0, 0, 0, 0);
////        sleep(100);
//
//        // back away from beacon
//        while (rangeReader.read(0x04, 2)[0] < 20 && opModeIsActive()) {
//            setDrive(0.1, 0.1, 0.1, 0.1);
//            addTelemetry();
//        }
//
//        setDrive(0, 0, 0, 0);
//        sleep(100);
//
//        //turn towards center goal
//        while (Math.abs(navx.getYaw() + 45) > 3 && opModeIsActive()){
//            if (Math.abs(navx.getYaw()) > 45) {
//                setDrive(0.4, 0, 0.4, 0);
//            } else {
//                setDrive(0.1, 0, 0.1, 0);
//            }
//            addTelemetry();
//        }
//
//        setDrive(0, 0, 0, 0);
//        sleep(100);
//
////        double currentFLencoder = FL.getCurrentPosition();
////        while (FL.getCurrentPosition() - currentFLencoder < -600){
////            setDrive(.4, .4, .4, .4);
////        }
////
////        setDrive(0, 0, 0, 0);
////        sleep(100);

        stop();
    }

    private void turnRobotCorner(int a){
        while (Math.abs(navx.getYaw() - a) > 2 && opModeIsActive()) {
            int scale;
            if (navx.getYaw() - a < 0) {
                //if robot has turned less than a degrees in left direction
                scale = -1;
            } else {
                //if robot has turned more than a degrees in left direction
                scale = 1;
            }
            if (Math.abs(navx.getYaw() - a) > 15){
                //if robot has turned less than (a-15) degrees in either direction
                //then turns robot at a faster speed
                setDrive(0, scale * 0.4);
            } else {
                //if robot has turned more than (a-15) degrees in either direction
                //turns robot at a slower speed
                setDrive(0, scale * 0.07);
            }
        }
        addTelemetry();
//        setDrive(0);
//        sleep(100);
    }

}
