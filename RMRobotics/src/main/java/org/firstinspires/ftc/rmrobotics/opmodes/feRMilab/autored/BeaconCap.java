package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autored;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;

import static org.firstinspires.ftc.rmrobotics.util.enums.Direction.BACKWARD;
import static org.firstinspires.ftc.rmrobotics.util.enums.Direction.CENTER;
import static org.firstinspires.ftc.rmrobotics.util.enums.Direction.LEFT;
import static org.firstinspires.ftc.rmrobotics.util.enums.Direction.RIGHT;
import static org.firstinspires.ftc.rmrobotics.util.enums.Drive.RANGE;
import static org.firstinspires.ftc.rmrobotics.util.enums.Drive.TIME;

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
//        drive(TIME,300,0.3); // TODO: hardcode correction for overshooting
//        setDrive(0);

        // turn left towards beacon
        turn(CENTER, -90, 0.2); // TODO: find correct turning angle

        // drive forward until close enough to beacon
        driveStop(RANGE, 11, 0.1); // or 10

        // detect colors
        boolean detected = false;
        double initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 200 && opModeIsActive()) {
            if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {
                left = Color.RED;
            } else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {
                left = Color.BLUE;
            } else {
                left = Color.NEITHER;
            }
            if (Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {
                right = Color.RED;
            } else if (Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {
                right = Color.BLUE;
            } else {
                right = Color.NEITHER;
            }

            // move beacon pusher arm to appropriate location
            if (left == Color.RED && right == Color.BLUE) {
                //left is red, right is blue
                detected = true;
                beacon = LEFT;
            } else if (left == Color.BLUE && right == Color.RED) {
                detected = true;
                beacon = RIGHT;
            } else {
                if (left == Color.RED || right == Color.BLUE) {
                    detected = true;
                    beacon = LEFT;
                } else if (left == Color.BLUE || right == Color.RED) {
                    detected = true;
                    beacon = RIGHT;
                }
                // telemetry
            }
        }

        // drive forward to hit beacon
        if (detected) {
//            sleep(100);
//            drive(TIME, 1500, -0.15); // TODO: finding right time and power!
            sleep(100);
            initTime = runtime.milliseconds();
            while (runtime.milliseconds() - initTime < 1500) { // run loop for 2 seconds
                if (runtime.milliseconds() - initTime < 1000) { // push out beacon pusher for 1 second
                    if (beacon == LEFT) {
                        pushLeft.setPower(1.0);
                    } else {
                        pushRight.setPower(1.0);
                    }
                    setDrive(-0.04);
                } else {
                    pushLeft.setPower(0);
                    pushRight.setPower(0);
                    setDrive(0);
                }
//                if (runtime.milliseconds() - initTime > 500) { // start driving towards wall at .5 seconds
//                    setDrive(0.07);
//                } else {
//                    setDrive(0);
//                }
            }
        }

        // back away from beacon
        driveStop(RANGE, 30, 0.2);
        // swingArm.setPosition(0.5);

        initTime = runtime.milliseconds();

        // retract beacon pusher motors
        if (beacon == LEFT) {
            pushLeft.setPower(-.7);
        }
        else {
            pushRight.setPower(-.7);
        }

        // SHOOTING
        while(runtime.milliseconds()-initTime < 4000 && opModeIsActive()) {
            // voltage = flyMC.getVoltage()*-0.1242 + 2.421;
            flyL.setPower(0.93);
            flyR.setPower(0.93);
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
            if (runtime.milliseconds()-initTime > 750) { // stop retracting motors after 750 seconds
                pushLeft.setPower(0);
                pushRight.setPower(0);
            }
        }

        belt.setPower(0);
        index.setPosition(.1);
        flyL.setPower(0);
        flyR.setPower(0);

        // FIRST BEACON DONE

        // turn towards second line
        turn(CENTER, -5, 0.15);

        // drive forward slightly to move center color sensor off the first line
        drive(TIME, 500, -0.6);

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

//        drive(TIME,100,-0.3);
//        setDrive(0);

        //turn left towards beacon
        turn(CENTER, -90, 0.2);

        // drive forward until close enough to beacon
        driveStop(RANGE, 11, 0.1); // or 10

        // detect colors
        detected = false;
        initTime = runtime.milliseconds();
        while (runtime.milliseconds() - initTime < 200 && opModeIsActive()) {
            if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 10) <= 1) {
                left = Color.RED;
            } else if (Math.abs(colorLeftReader.read(0x04, 1)[0] - 3) <= 1) {
                left = Color.BLUE;
            } else {
                left = Color.NEITHER;
            }
            if (Math.abs(colorRightReader.read(0x04, 1)[0] - 10) <= 1) {
                right = Color.RED;
            } else if (Math.abs(colorRightReader.read(0x04, 1)[0] - 3) <= 1) {
                right = Color.BLUE;
            } else {
                right = Color.NEITHER;
            }

            // move beacon pusher arm to appropriate location
            if (left == Color.RED && right == Color.BLUE) {
                //left is red, right is blue
                detected = true;
                beacon = LEFT;
            } else if (left == Color.BLUE && right == Color.RED) {
                detected = true;
                beacon = RIGHT;
            } else {
                if (left == Color.RED || right == Color.BLUE) {
                    detected = true;
                    beacon = LEFT;
                } else if (left == Color.BLUE || right == Color.RED) {
                    detected = true;
                    beacon = RIGHT;
                }
                // telemetry
            }
        }

        // drive forward to hit beacon
        if (detected) {
//            sleep(100);
//            drive(TIME, 1500, -0.15); // TODO: finding right time and power!
            sleep(100);
            initTime = runtime.milliseconds();
            while (runtime.milliseconds() - initTime < 1500) { // run loop for 2 seconds
                if (runtime.milliseconds() - initTime < 1000) { // push out beacon pusher for 1 second
                    if (beacon == LEFT) {
                        pushLeft.setPower(1.0);
                    } else {
                        pushRight.setPower(1.0);
                    }
                    setDrive(-0.05);
                } else {
                    pushLeft.setPower(0);
                    pushRight.setPower(0);
                    setDrive(0);
                }
//                if (runtime.milliseconds() - initTime > 500) { // start driving towards wall at .5 seconds
//                    setDrive(0.07);
//                } else {
//                    setDrive(0);
//                }
            }
        }

        sleep(200);

        // back away from beacon
        drive(RANGE, 20, 0.2);
        swingArm.setPosition(0.5);

        // END of BEACON 2

        turn(CENTER, -55, 0.4);

        initTime = runtime.milliseconds();

        // retract beacon pusher motors
        if (beacon == LEFT) {
            pushLeft.setPower(-.7);
        }
        else {
            pushRight.setPower(-.7);
        }


        //turn towards center goal
        while(runtime.milliseconds()-initTime < 4000 && opModeIsActive()) {
            if (runtime.milliseconds()-initTime < 1500) {
                setDrive(0.7);
            } else {
                setDrive(0);
            }
            if (runtime.milliseconds()-initTime > 750) { // stop retracting motors after 750 seconds
                pushLeft.setPower(0);
                pushRight.setPower(0);
            }
        }
        stop(); // testing use only
//        setDrive(0);

        //drive to knock off cap ball
//        drive(TIME, 2200, 0.4);
//        setEnc(1100); // TODO: check if 1100 is perfect on red (works on blue)
//        setDrive(0.5);
        while (opModeIsActive()) {
            telemetry.addData("RED", "WINS");
            telemetry.update();
        }

        stop();
    }
}
