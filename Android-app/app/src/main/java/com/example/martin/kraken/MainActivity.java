package com.example.martin.kraken;

import android.content.Context;
import android.os.Bundle;
import android.widget.TextView;

import java.io.IOException;
import java.io.InputStream;


import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;

import io.github.controlwear.virtual.joystick.android.JoystickView;

import java.io.OutputStream;
import java.net.Socket;
import java.net.UnknownHostException;
import java.util.List;

import android.os.AsyncTask;
import android.app.Activity;
import android.widget.EditText;
import android.widget.ToggleButton;

import static java.lang.Math.abs;
import static java.lang.Math.atan2;

public class MainActivity extends Activity {

    TextView textResponse, textData;
    EditText editTextAddress, editTextPort;
    ToggleButton buttonConnect, toggleButton_use_joystick, toggleButton_pump_low, toggleButton_pump_high, toggleButton_piston_low1, toggleButton_piston_low2, toggleButton_piston_high;
    int robot_angle, robot_strength;
    int received_buffer[], sent_buffer[];

    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        editTextAddress = findViewById(R.id.address);
        editTextPort = findViewById(R.id.port);
        buttonConnect = findViewById(R.id.connect);
        textResponse = findViewById(R.id.response);
        toggleButton_use_joystick = findViewById(R.id.use_joystick);
        toggleButton_pump_low = findViewById(R.id.pump_low);
        toggleButton_pump_high = findViewById(R.id.pump_high);
        toggleButton_piston_high = findViewById(R.id.piston_high);
        toggleButton_piston_low1 = findViewById(R.id.piston_low1);
        toggleButton_piston_low2 = findViewById(R.id.piston_low2);
        textData = findViewById(R.id.data);

        toggleButton_use_joystick.setChecked(true);

        toggleButton_use_joystick.setTextOn("Joystick mode");
        toggleButton_use_joystick.setTextOff("Tilt mode");
        toggleButton_use_joystick.setText("Joystick mode");

        toggleButton_piston_low1.setTextOff("Piston low 1");
        toggleButton_piston_low1.setTextOn("Piston low 1");
        toggleButton_piston_low1.setText("Piston low 1");

        toggleButton_piston_low2.setTextOff("Piston low 2");
        toggleButton_piston_low2.setTextOn("Piston low 2");
        toggleButton_piston_low2.setText("Piston low 2");

        toggleButton_piston_high.setTextOff("Piston high");
        toggleButton_piston_high.setTextOn("Piston high");
        toggleButton_piston_high.setText("Piston high");

        toggleButton_pump_low.setTextOff("Pump low");
        toggleButton_pump_low.setTextOn("Pump low");
        toggleButton_pump_low.setText("Pump low");

        toggleButton_pump_high.setTextOff("Pump high");
        toggleButton_pump_high.setTextOn("Pump high");
        toggleButton_pump_high.setText("Pump high");

        buttonConnect.setTextOff("Disconnected");
        buttonConnect.setText("Disconnected");
        buttonConnect.setTextOn("Connected");

        editTextAddress.setText("192.168.4.1");
        editTextPort.setText("5550");

        received_buffer = new int[4];
        sent_buffer = new int[4];

        JoystickView joystick = findViewById(R.id.joystickView);
        joystick.setOnMoveListener(new JoystickView.OnMoveListener() {

            @Override
            public void onMove(int angle, int strength) {
                if (toggleButton_use_joystick.isChecked()) {
                    robot_angle = angle - 90;
                    robot_strength = strength;
                }
            }
        });

        SensorManager sensorManager = (SensorManager) getSystemService(Context.SENSOR_SERVICE);
        if (sensorManager != null) {
            List<Sensor> ls =  sensorManager.getSensorList(SensorManager.SENSOR_ALL);

            Sensor accelerometre = sensorManager.getDefaultSensor(Sensor.TYPE_ACCELEROMETER);
            if(accelerometre != null) {
                boolean b = sensorManager.registerListener(mSensorEventListener,accelerometre,1000);
            }
        }

    }

    public class MyClientTask extends AsyncTask<Void, Void, Void> {

        String dstAddress;
        int dstPort;
        int response;
        byte[] msent_buffer, m_received_buffer;

        MyClientTask(String addr, int port){
            dstAddress = addr;
            dstPort = port;
        }

        @Override
        protected Void doInBackground(Void... arg0) {
            try {
                Socket socket = new Socket(dstAddress, dstPort);
                if (socket.isConnected()) {
                    OutputStream out = socket.getOutputStream();
                    InputStream in = socket.getInputStream();
                    msent_buffer = new byte[4];
                    if (buttonConnect.isChecked()) {
                        msent_buffer[3] = (byte) 1;
                    } else {
                        msent_buffer[3] = 0;
                    }
                    if (toggleButton_piston_high.isChecked()) {
                        msent_buffer[3] += (int) 32;
                    }
                    if (toggleButton_piston_low1.isChecked()) {
                        msent_buffer[3] += (int) 16;
                    }
                    if (toggleButton_piston_low2.isChecked()) {
                        msent_buffer[3] += (int) 8;
                    }
                    if (toggleButton_pump_high.isChecked()) {
                        msent_buffer[3] += (int) 4;
                    }
                    if (toggleButton_pump_low.isChecked()) {
                        msent_buffer[3] += (int) 2;
                    }

                    msent_buffer[2] = (byte) (robot_angle / 256);
                    msent_buffer[1] = (byte) (robot_angle % 256);
                    msent_buffer[0] = (byte) robot_strength;
                    out.write(msent_buffer,0, msent_buffer.length);
                    m_received_buffer = new byte[4];
                    in.read(m_received_buffer, 0, m_received_buffer.length);
                    out.flush();
                    out.close();
                    in.close();
                    socket.close();
                }

            } catch (UnknownHostException e) {
                e.printStackTrace();
            } catch (IOException e) {
                e.printStackTrace();
            }

            return null;
        }

        @Override
        protected void onPostExecute(Void result) {
            try {
                for (int i=0; i<4; i++) {
                    received_buffer[i] = (int) m_received_buffer[i];
                    sent_buffer[i] = (int) msent_buffer[i];
                }
            } catch (Exception e) {}
            super.onPostExecute(result);
        }

    }

    final SensorEventListener mSensorEventListener = new SensorEventListener() {
        public void onAccuracyChanged(Sensor sensor, int accuracy) {
            // Que faire en cas de changement de précision ?
        }

        public void onSensorChanged(SensorEvent sensorEvent) {
            if (!toggleButton_use_joystick.isChecked()) {
                robot_strength = (int) (100 - abs(sensorEvent.values[2] / 0.0981f)) * 3;
                if (robot_strength < 10) {
                    robot_strength = 0;
                } else if (robot_strength > 100) {
                    robot_strength = 100;
                }
                robot_angle = (int) (90 + atan2(sensorEvent.values[0], -sensorEvent.values[1]) * 180 / 3.141592);
            }
            if (robot_angle < 0) {
                robot_angle += 360;
            }
            textData.setText("Angle: " + robot_angle+ "\nStrength: " + robot_strength);

            if (buttonConnect.isChecked()) {
                MyClientTask myClientTask = new MyClientTask(
                        editTextAddress.getText().toString(),
                        Integer.parseInt(editTextPort.getText().toString()));
                myClientTask.execute();

                String output = "", input = "";
                for (int i=0; i<4; i++) {
                    output += received_buffer[i] + "   ";
                    input += sent_buffer[i] + "   ";
                }
                textResponse.setText("Message to Kraken:    " + input + "\nMessage from Kraken:   " + output);
            }
        }
    };

}