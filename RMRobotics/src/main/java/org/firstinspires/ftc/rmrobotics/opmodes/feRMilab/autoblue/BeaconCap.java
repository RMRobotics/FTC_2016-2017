package org.firstinspires.ftc.rmrobotics.opmodes.feRMilab.autoblue;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.rmrobotics.core.FeRMiLinear;
import org.firstinspires.ftc.rmrobotics.util.Color;

import static org.firstinspires.ftc.rmrobotics.util.Direction.BACKWARD;
import static org.firstinspires.ftc.rmrobotics.util.Direction.CENTER;
import static org.firstinspires.ftc.rmrobotics.util.Direction.RIGHT;
import static org.firstinspires.ftc.rmrobotics.util.Drive.RANGE;
import static org.firstinspires.ftc.rmrobotics.util.Drive.TIME;

/**
 * Created by Simon on 1/6/16.
 */
// BLUE TEAM


@Autonomous(name = "BLUE: Beacon")
public class BeaconCap extends FeRMiLinear {

    @Override
    public void runOpMode() {
        super.initialize(Color.BLUE, DcMotor.RunMode.RUN_USING_ENCODER, BACKWARD);

        // turn towards first beacon
        turn(RIGHT, 42, 0.4);

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
        sleep(150);

        // drives backwards to correct for overshooting
//        while (colorCenterReader.read(0x08, 1)[0] < 25 && opModeIsActive()) {
//            setDrive(0.07);
//        }
        drive(TIME,200,0.3);
        setDrive(0);

        // turn right towards beacon
        turn(CENTER, 86, 0.2);

        sleep(100);

        // drive forward until close enough to beacon
        drive(RANGE, 17, 0.1);

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
                swingArm.setPosition(0);
                detected = true;
            } else if (left == Color.BLUE && right == Color.RED) {
                swingArm.setPosition(1.0);
                detected = true;
            } else {
                if (left == Color.RED || right == Color.BLUE) {
                    swingArm.setPosition(0);
                    detected = true;
                } else if (left == Color.BLUE || right == Color.RED) {
                    swingArm.setPosition(1.0);
                    detected = true;
                }
            }
        }

        // drive forward to hit beacon
        if (detected) {
            sleep(100);
            drive(TIME, 1500, -0.15);
        }

        // back away from beacon
        drive(RANGE, 30, 0.2);
        swingArm.setPosition(0.5);

//        double power;
//        double voltageLimit = 0;
//        if (voltage < voltageLimit){
//            power = 1;
//        }
//        else{
//            power = .93;
//        }

        initTime = runtime.milliseconds();
        while(runtime.milliseconds()-initTime < 4000 && opModeIsActive()) {
            // voltage = flyMC.getVoltage()*-0.1242 + 2.421 + 0.;

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
        }

        belt.setPower(0);
        index.setPosition(.1);
        flyL.setPower(0);
        flyR.setPower(0);

        // FIRST BEACON DONE

        // turn towards second line
        turn(CENTER, 7, 0.15);

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

        drive(TIME,100,-0.3);
        setDrive(0);

        //turn right towards beacon
        turn(CENTER, 90, 0.2);

        sleep(100);

        // drive forward until close enough to beacon
        drive(RANGE, 14, 0.1);

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
                swingArm.setPosition(0);
                detected = true;
            } else if (left == Color.BLUE && right == Color.RED) {
                swingArm.setPosition(1.0);
                detected = true;
            } else {
                if (left == Color.RED || right == Color.BLUE) {
                    swingArm.setPosition(0);
                    detected = true;
                } else if (left == Color.BLUE || right == Color.RED) {
                    swingArm.setPosition(1.0);
                    detected = true;
                }
            }
        }

        // drive forward to hit beacon
        if (detected) {
            sleep(100);
            drive(TIME, 1500, -0.15);
        }

        // back away from beacon
        drive(RANGE, 20, .1);
        swingArm.setPosition(0.5);

        //turn towards center goal
        turn(CENTER, 50, 0.4);

        sleep(100);

        // drive to knock off cap ball
        drive(TIME, 2200, 0.4);
        //setEnc(1100); // TODO: check if 1100 is perfect (previously 1000)
        //setDrive(0.5);
        while (Math.abs(FL.getCurrentPosition() - FL.getTargetPosition()) > 10 && opModeIsActive()) {
            telemetry.addData("BLUE", "WINS");
            telemetry.update();
        }

        setDrive(0);

        stop();
    }
}
