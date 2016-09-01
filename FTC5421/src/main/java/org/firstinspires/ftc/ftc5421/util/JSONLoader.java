package org.firstinspires.ftc.ftc5421.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftc5421.hardware.MotorType;
import org.firstinspires.ftc.ftc5421.hardware.RMotor;
import org.firstinspires.ftc.ftc5421.hardware.RServo;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

public class JSONLoader {

    private Map<String, RMotor> motorImportMap = new HashMap<String, RMotor>();
    private Map<String, RServo> servoImportMap =  new HashMap<String, RServo>();
    private final HardwareMap hardwareMap;

    public JSONLoader(String filePath, final HardwareMap hMap) throws IOException, ParseException {
        hardwareMap = hMap;
        JSONParser jsonParser = new JSONParser();

        JSONObject jsonFile = (JSONObject) jsonParser.parse(loadJSONFromAsset(filePath));
        JSONArray jsonMotors = (JSONArray) jsonFile.get("motors");
        JSONArray jsonServos = (JSONArray) jsonFile.get("servos");
        configureMotors(jsonMotors);
        configureServos(jsonServos);
    }

    public String loadJSONFromAsset(String path) {
        String json;
        try {
            InputStream is = hardwareMap.appContext.getAssets().open(path);
            int size = is.available();
            byte[] buffer = new byte[size];
            is.read(buffer);
            is.close();
            json = new String(buffer, "UTF-8");
        } catch (IOException ex) {
            ex.printStackTrace();
            return null;
        }
        return json;
    }

    private void configureMotors(JSONArray JSONMotors) {
        for (Object mObj : JSONMotors) {
            JSONObject mJSON = (JSONObject) mObj;
            String motorName = (String) mJSON.get("name");
            double minPower = (Double) mJSON.get("minPower");
            double maxPower = (Double) mJSON.get("maxPower");
            DcMotor dcParent = hardwareMap.dcMotor.get(motorName);
            DcMotor.Direction d = stringToMotorDirection((String) mJSON.get("direction"));
            MotorType mT = stringToMotorType((String) mJSON.get("motorType"));
            RMotor m = new RMotor(dcParent, d, minPower, maxPower, mT);
            motorImportMap.put(motorName, m);
        }
    }

    private void configureServos(JSONArray JSONServos) {
        for (Object sObj : JSONServos) {
            JSONObject sJSON = (JSONObject) sObj;
            String servoName = (String) sJSON.get("name");
            double minPosition = (Double) sJSON.get("minPosition");
            double maxPosition = (Double) sJSON.get("maxPosition");
            Servo sParent = hardwareMap.servo.get(servoName);
            Servo.Direction d = stringToServoDirection((String) sJSON.get("direction"));
            double init = (Double) sJSON.get("init");
            RServo s = new RServo(sParent, d, minPosition, maxPosition, init);
            servoImportMap.put(servoName, s);
        }
    }

    private DcMotor.Direction stringToMotorDirection(String stringD) { //ToDo check if valueOf works as expected
        if (stringD.equals("FORWARD")) {
            return DcMotor.Direction.FORWARD;
        } else if (stringD.equals("REVERSE")) {
            return DcMotor.Direction.REVERSE;
        } else {
            return DcMotor.Direction.valueOf(stringD);
        }
    }

    private Servo.Direction stringToServoDirection(String stringD) {
        if (stringD.equals("FORWARD")) {
            return Servo.Direction.FORWARD;
        } else if (stringD.equals("REVERSE")) {
            return Servo.Direction.REVERSE;
        } else {
            return Servo.Direction.valueOf(stringD);
        }
    }

    private MotorType stringToMotorType(String stringD) {
        if (stringD.equals("NVRST_20")) {
            return MotorType.NVRST_20;
        } else if (stringD.equals("NVRST_40")) {
            return MotorType.NVRST_40;
        } else if (stringD.equals("NVRST_60")) {
            return MotorType.NVRST_60;
        } else {
            return MotorType.TETRIX;
        }
    }

    public Map<String,RMotor> getMotorMap() {
        return motorImportMap;
    }

    public Map<String, RServo> getServoMap() {
        return servoImportMap;
    }

}
