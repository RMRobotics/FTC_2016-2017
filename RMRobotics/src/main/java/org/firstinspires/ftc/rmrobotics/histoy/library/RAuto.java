package org.firstinspires.ftc.rmrobotics.histoy.library;

import com.kauailabs.navx.ftc.AHRS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.rmrobotics.control.RControl;
import org.firstinspires.ftc.rmrobotics.hardware.i2csensor;
import org.firstinspires.ftc.rmrobotics.hardware.crservo;
import org.firstinspires.ftc.rmrobotics.hardware.motor;
import org.firstinspires.ftc.rmrobotics.hardware.servo;
import org.firstinspires.ftc.rmrobotics.util.config.Robot;
import org.firstinspires.ftc.rmrobotics.util.enums.Color;

import java.util.ArrayList;

public abstract class RAuto extends LinearOpMode {

    private ArrayList<motor> motors = new ArrayList<>();
    private ArrayList<servo> servos =  new ArrayList<>();
    private ArrayList<crservo> crservos = new ArrayList<>();
    private ArrayList<i2csensor> sensors = new ArrayList<>();
    protected ElapsedTime runtime = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
    protected DeviceInterfaceModule dim;
    protected AHRS navx;
    protected RControl control;
    protected Robot robot;
    protected Color alliance;

    public void initialize() {
        configureHardware();
        dim = hardwareMap.deviceInterfaceModule.get("dim");

        for (motor m : motors) {
            m.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            sleep(200);
            m.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }
        for (servo s : servos) {
            s.setInitPos();
        }
        for (i2csensor i : sensors) {
            i.engage();
        }
        try {
            while (navx.isCalibrating()) {
                telemetry.addData("Status", !navx.isCalibrating());
                telemetry.update();
            }
        } catch (Exception e) {
            telemetry.addData("Error:", "no navx object, " + e.getMessage());
            telemetry.update();
        }

        switch (alliance) {
            case RED:
                dim.setLED(0, false); // blue
                dim.setLED(1, true); // red
                break;
            case BLUE:
                dim.setLED(1, false);
                dim.setLED(0, true);
                break;
            case NEITHER:
                dim.setLED(1, false);
                dim.setLED(1, true);
                break;
            default:
                dim.setLED(0, false);
                dim.setLED(0, true);
                break;
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        runtime.reset(); // reset runtime counter
        navx.zeroYaw(); // reset navx yaw value
    }

    protected void configureHardware() {
        robot = setRobot();
        motors = robot.motors();
        servos = robot.servos();
        crservos = robot.crservos();
    }

    protected abstract Robot setRobot();
    protected abstract void setAlliance();

    protected void addTelemetry() {
        telemetry.addData("TIME", runtime.time());
        for (motor m : motors) {
            telemetry.addData(String.valueOf(hardwareMap.getNamesOf(m.getParent())), m.getPower());
        }
        for (servo s : servos) {
            telemetry.addData(String.valueOf(hardwareMap.getNamesOf(s.getParent())), s.getPosition());
        }
        for (crservo c : crservos) {
            telemetry.addData(String.valueOf(hardwareMap.getNamesOf(c.getParent())), c.getPower());
        }
    }

}
