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

        // turn left towards first beacon
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

        // turn left towards beacon
        turn(CENTER, -87, 0.2); // TODO: find correct turning angle

        // drive forward until close enough to beacon
        driveStop(RANGE, 12, 0.1); // or 10

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

            // determine side with correct color
            if (left == Color.RED && right == Color.BLUE) {
                //left is red, right is blue
                detected = true;
                beacon = LEFT;
                telemetry.addData("SURE", beacon.toString());
            } else if (left == Color.BLUE && right == Color.RED) {
                detected = true;
                beacon = RIGHT;
                telemetry.addData("SURE", beacon.toString());
            } else {
                if (left == Color.RED || right == Color.BLUE) {
                    detected = true;
                    beacon = LEFT;
                } else if (left == Color.BLUE || right == Color.RED) {
                    detected = true;
                    beacon = RIGHT;
                }
                // output probable side with correct color
                telemetry.addData("UNSURE:", beacon.toString());
            }
            telemetry.update();
        }

        // drive forward to hit beacon
        if (detected) {
            sleep(100);
            initTime = runtime.milliseconds();
            while (runtime.milliseconds() - initTime < 1700) { // run loop for 1.7 seconds
                if (runtime.milliseconds() - initTime < 1000) { // push out beacon pusher for 1 second
                    if (beacon == LEFT) {
                        pushLeft.setPower(1.0);
                    } else {
                        pushRight.setPower(1.0);
                    }
                    setDrive(-0.07);
                } else {
                    pushLeft.setPower(0);
                    pushRight.setPower(0);
                    setDrive(0);
                }
            }
        }
        sleep(70);

        // begin flywheel spin up
        flyL.setPower(1.0);
        flyR.setPower(1.0);

        // back away from beacon
        driveStop(RANGE, 45, 0.2);

        // FIRST BEACON DONE

        initTime = runtime.milliseconds();

        // retract beacon pusher motors
        if (beacon == LEFT) {
            pushLeft.setPower(-.7);
        }
        else {
            pushRight.setPower(-.7);
        }

        // SHOOTING
        turn(CENTER, -92, 0.2);
        while(runtime.milliseconds()-initTime < 3000 && opModeIsActive()) {
            if (runtime.milliseconds() - initTime > 500) {
                index.setPosition(.5);
                belt.setPower(1.0);
            } else if (runtime.milliseconds() - initTime > 1500) {
                index.setPosition(.5);
                belt.setPower(0);
            } else if (runtime.milliseconds() - initTime > 2000) {
                index.setPosition(.5);
                belt.setPower(1);
            }
            // stop retracting motors after 750 seconds
            if (runtime.milliseconds()-initTime > 750) {
                pushLeft.setPower(0);
                pushRight.setPower(0);
            }
        }

        belt.setPower(0);
        index.setPosition(.1);
        flyL.setPower(0);
        flyR.setPower(0);
        liftHold.setPosition(0.03);

        // turn left towards second line
        turn(CENTER, -10, 0.15);

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

        // drive backwards to correct for overshooting
        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
            setDrive(0.04);
        }
        setDrive(0);

        // turn left towards beacon
        turn(CENTER, -89, 0.2);

        // drive forward until close enough to beacon
        drive(TIME, 200, -0.1);
        driveStop(RANGE, 12, 0.1); // or 10

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

            // determine side with correct color
            if (left == Color.RED && right == Color.BLUE) {
                //left is red, right is blue
                detected = true;
                beacon = LEFT;
                telemetry.addData("SURE", beacon.toString());
            } else if (left == Color.BLUE && right == Color.RED) {
                detected = true;
                beacon = RIGHT;
                telemetry.addData("SURE", beacon.toString());
            } else {
                if (left == Color.RED || right == Color.BLUE) {
                    detected = true;
                    beacon = LEFT;
                } else if (left == Color.BLUE || right == Color.RED) {
                    detected = true;
                    beacon = RIGHT;
                }
                // output probable side with correct color
                telemetry.addData("UNSURE:", beacon.toString());
            }
            telemetry.update();
        }

        // drive forward to hit beacon
        if (detected) {
            sleep(100);
            initTime = runtime.milliseconds();
            while (runtime.milliseconds() - initTime < 1700) { // run loop for 2 seconds
                if (runtime.milliseconds() - initTime < 1000) { // push out beacon pusher for 1 second
                    if (beacon == LEFT) {
                        pushLeft.setPower(1.0);
                    } else {
                        pushRight.setPower(1.0);
                    }
                    setDrive(-0.07);
                } else {
                    pushLeft.setPower(0);
                    pushRight.setPower(0);
                    setDrive(0);
                }
            }
        }

        sleep(70);

        // back away from beacon
        drive(RANGE, 20, 0.2);

        // SECOND BEACON DONE

        // turn towards center goal
        turn(CENTER, -50, 0.4);

        initTime = runtime.milliseconds();

        // retract beacon pusher motors
        if (beacon == LEFT) {
            pushLeft.setPower(-.7);
        }
        else {
            pushRight.setPower(-.7);
        }

        // drive onto center goal
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

        while (opModeIsActive()) {
            telemetry.addData("RED", "WINS");
            telemetry.update();
        }

        stop();
    }
}