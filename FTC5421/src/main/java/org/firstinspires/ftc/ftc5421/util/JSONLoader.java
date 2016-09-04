package org.firstinspires.ftc.ftc5421.util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.ftc5421.hardware.crservo;
import org.firstinspires.ftc.ftc5421.hardware.motor;
import org.firstinspires.ftc.ftc5421.hardware.servo;
import org.json.simple.JSONArray;
import org.json.simple.JSONObject;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

import java.io.IOException;
import java.io.InputStream;
import java.util.HashMap;
import java.util.Map;

public class JSONLoader {

    private Map<String, motor> motorImportMap = new HashMap<String, motor>();
    private Map<String, servo> servoImportMap =  new HashMap<String, servo>();
    private Map<String, crservo> crservoImportMap = new HashMap<String, crservo>();
    private final HardwareMap hardwareMap;

    public JSONLoader(String filePath, final HardwareMap hMap) throws IOException, ParseException {
        hardwareMap = hMap;
        JSONParser jsonParser = new JSONParser();

        JSONObject jsonFile = (JSONObject) jsonParser.parse(loadJSONFromAsset(filePath));
        JSONArray jsonMotors = (JSONArray) jsonFile.get("motors");
        JSONArray jsonServos = (JSONArray) jsonFile.get("servos");
        JSONArray jsonCRServos = (JSONArray) jsonFile.get("crservos");
        configureMotors(jsonMotors);
        configureServos(jsonServos);
        configureCRServos(jsonCRServos);
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
            DcMotor.RunMode r = DcMotor.RunMode.valueOf((String) mJSON.get("runMode"));
            DcMotor.ZeroPowerBehavior z = DcMotor.ZeroPowerBehavior.valueOf((String) mJSON.get("zeroMode")); //ToDo check if valueOf actually works
            DcMotor dcParent = hardwareMap.dcMotor.get(motorName);
            DcMotor.Direction d = DcMotor.Direction.valueOf((String) mJSON.get("direction"));
            MotorType mT = MotorType.toType((int) mJSON.get("motorType"));
            motor m = new motor(dcParent, d, r, z, mT);
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
            Servo.Direction d = Servo.Direction.valueOf((String) sJSON.get("direction"));
            double init = (Double) sJSON.get("init");
            servo s = new servo(sParent, d, minPosition, maxPosition, init);
            servoImportMap.put(servoName, s);
        }
    }

    private void configureCRServos(JSONArray JSONCRServos) {
        for (Object cObj : JSONCRServos) {
            JSONObject cJSON = (JSONObject) cObj;
            String crservoName = (String) cJSON.get("name");
            CRServo cParent = hardwareMap.crservo.get(crservoName);
            CRServo.Direction d = CRServo.Direction.valueOf((String) cJSON.get("direction"));
            crservo c = new crservo(cParent, d);
            crservoImportMap.put(crservoName, c);
        }
    }

    public Map<String,motor> getMotorMap() {
        return motorImportMap;
    }

    public Map<String, servo> getServoMap() {
        return servoImportMap;
    }

}
