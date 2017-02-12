package org.firstinspires.ftc.rmrobotics.opmodes.AutoNav;


import android.view.View;
import android.view.ViewGroup;
import android.widget.Button;
import android.widget.LinearLayout;
import android.widget.RadioButton;
import android.widget.RadioGroup;
import android.widget.Space;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcontroller.internal.FtcRobotControllerActivity;

/**
 * Created by Peter on 1/24/2017.
 */

@Autonomous(name = "AutonomousConfig", group = "AutoConf")
public class AutonomousConfig extends LinearOpMode {

    static LinearLayout layout;
    volatile boolean  save = false;
    volatile boolean isRed = false;
    volatile String firstBeacon = "";
    volatile String secondBeacon = "";
    volatile boolean isRight = false;


    @Override
    public void runOpMode() throws InterruptedException {

        //the layout on which you are working
        FtcRobotControllerActivity act = ((FtcRobotControllerActivity) hardwareMap.appContext);

        final AutoNavConfig cfg = new AutoNavConfig();
        cfg.ReadConfig(act);



        layout = (LinearLayout)act.findViewById(com.qualcomm.ftcrobotcontroller.R.id.cameraMonitorViewId);


        //add button to the layout
        ((FtcRobotControllerActivity) hardwareMap.appContext).runOnUiThread(new Runnable() {
            @Override
            public void run() {


                final Space sp = new Space(layout.getContext());
                ViewGroup.LayoutParams height = new LinearLayout.LayoutParams(10, 200);;
                sp.setLayoutParams(height);
                layout.addView(sp);//you add the whole RadioGroup to the layout

                // Color
                final RadioButton[] colorBtn = new RadioButton[2];
                final RadioGroup rg = new RadioGroup(layout.getContext()); //create the RadioGroup
                rg.setOrientation(RadioGroup.HORIZONTAL);//or RadioGroup.VERTICAL
                colorBtn[0] = new RadioButton(layout.getContext());
                colorBtn[0].setText("Red");
                colorBtn[1] = new RadioButton(layout.getContext());
                colorBtn[1].setText("Blue");
                rg.addView(colorBtn[0]);
                rg.addView(colorBtn[1]);
                layout.addView(rg);//you add the whole RadioGroup to the layout
                if(cfg.isRed) colorBtn[0].setChecked(true);
                else colorBtn[1].setChecked(true);

                // First Beacon
                final RadioButton[] fbBtn = new RadioButton[4];
                final RadioGroup fbrg = new RadioGroup(layout.getContext()); //create the RadioGroup
                fbrg.setOrientation(RadioGroup.HORIZONTAL);//or RadioGroup.VERTICAL
                fbBtn[0] = new RadioButton(layout.getContext());
                fbBtn[0].setText("W");
                fbBtn[1] = new RadioButton(layout.getContext());
                fbBtn[1].setText("T");
                fbBtn[2] = new RadioButton(layout.getContext());
                fbBtn[2].setText("L");
                fbBtn[3] = new RadioButton(layout.getContext());
                fbBtn[3].setText("G");
                fbrg.addView(fbBtn[0]);
                fbrg.addView(fbBtn[1]);
                fbrg.addView(fbBtn[2]);
                fbrg.addView(fbBtn[3]);
                layout.addView(fbrg);
                switch(cfg.firstBeacon) {
                    case "Wheels": fbBtn[0].setChecked(true); break;
                    case "Tools": fbBtn[1].setChecked(true); break;
                    case "Legos": fbBtn[2].setChecked(true); break;
                    case "Gears": fbBtn[3].setChecked(true); break;
                }

                // Second Beacon
                final RadioButton[] sbBtn = new RadioButton[4];
                final RadioGroup sbrg = new RadioGroup(layout.getContext()); //create the RadioGroup
                sbrg.setOrientation(RadioGroup.HORIZONTAL);//or RadioGroup.VERTICAL
                sbBtn[0] = new RadioButton(layout.getContext());
                sbBtn[0].setText("W");
                sbBtn[1] = new RadioButton(layout.getContext());
                sbBtn[1].setText("T");
                sbBtn[2] = new RadioButton(layout.getContext());
                sbBtn[2].setText("L");
                sbBtn[3] = new RadioButton(layout.getContext());
                sbBtn[3].setText("G");
                sbrg.addView(sbBtn[0]);
                sbrg.addView(sbBtn[1]);
                sbrg.addView(sbBtn[2]);
                sbrg.addView(sbBtn[3]);
                layout.addView(sbrg);//you add the whole RadioGroup to the layout
                switch(cfg.secondBeacon) {
                    case "Wheels": sbBtn[0].setChecked(true); break;
                    case "Tools": sbBtn[1].setChecked(true); break;
                    case "Legos": sbBtn[2].setChecked(true); break;
                    case "Gears": sbBtn[3].setChecked(true); break;
                }

                // Direction
                final RadioButton[] dirBtn = new RadioButton[2];
                final RadioGroup dirrg = new RadioGroup(layout.getContext()); //create the RadioGroup
                dirrg.setOrientation(RadioGroup.HORIZONTAL);//or RadioGroup.VERTICAL
                dirBtn[0] = new RadioButton(layout.getContext());
                dirBtn[0].setText("Right");
                dirBtn[1] = new RadioButton(layout.getContext());
                dirBtn[1].setText("Left");
                dirrg.addView(dirBtn[0]);
                dirrg.addView(dirBtn[1]);
                layout.addView(dirrg);//you add the whole RadioGroup to the layout
                if(cfg.isRight) dirBtn[0].setChecked(true);
                else dirBtn[1].setChecked(true);


                //set the properties for button
                final Button btnSave = new Button(layout.getContext());
                btnSave.setLayoutParams(new LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT));
                btnSave.setText("Save");
                btnSave.setOnClickListener(new View.OnClickListener() {

                    public void onClick(View v) {
                        if(colorBtn[0].isChecked()) isRed = true;
                        else isRed = false;

                        if(fbBtn[0].isChecked()) firstBeacon = "Wheels";
                        else if(fbBtn[1].isChecked()) firstBeacon = "Tools";
                        else if(fbBtn[2].isChecked()) firstBeacon = "Legos";
                        else if(fbBtn[3].isChecked()) firstBeacon = "Gears";

                        if(sbBtn[0].isChecked()) secondBeacon = "Wheels";
                        else if(sbBtn[1].isChecked()) secondBeacon = "Tools";
                        else if(sbBtn[2].isChecked()) secondBeacon = "Legos";
                        else if(sbBtn[3].isChecked()) secondBeacon = "Gears";

                        if(dirBtn[0].isChecked()) isRight = true;
                        else isRight = false;

                        layout.removeView(dirrg);
                        layout.removeView(sp);
                        layout.removeView(sbrg);
                        layout.removeView(fbrg);
                        layout.removeView(rg);
                        layout.removeView(btnSave);

                        save = true;

                    }
                });
                layout.addView(btnSave);
            }
        });


        while(save == false && !isStopRequested()) sleep(100);
        if(save) {

            cfg.isRed = isRed;
            cfg.firstBeacon = firstBeacon;
            cfg.secondBeacon = secondBeacon;
            cfg.isRight = isRight;
            cfg.WriteConfig(act);
            telemetry.clear();
            telemetry.addData("Update", " saved");
            telemetry.update();
        } else {
            telemetry.clear();
            telemetry.addData("Update", " cancelled");
            telemetry.update();
        }


        stop();
    }
}
