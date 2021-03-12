/*
   MainActivity.java : main activity class for Android version of Hackflight GCS

   Copyright (C) 2021 Rob Jones, Alec Singer, Simon D. Levy

   MIT License
 */

package edu.wlu.cs.levy.hackflight;

import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.TextView;
import android.widget.Toast;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.os.IBinder;
import android.os.Handler;
import android.os.Message;

import java.util.Set;

import edu.wlu.cs.msppg.ATTITUDE_Handler;
import edu.wlu.cs.msppg.Parser;

public class MainActivity extends AppCompatActivity {

    private static UsbService usbService;

    private MyHandler mHandler;

    private TextView mAttitudeText;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        mAttitudeText = (TextView)findViewById(R.id.attitude_Text);

        startService(UsbService.class, usbConnection, null);        //starts the usb service

        mHandler = new MyHandler(this);
    }

    @Override
    public void onResume() {
        super.onResume();
        setFilters();  // Start listening notifications from UsbService
        startService(UsbService.class, usbConnection, null); // Start UsbService(if it was not started before) and Bind it
    }

    public void setAttitudeText(int roll, int pitch, int yaw) {

        mAttitudeText.setText(String.format("Roll: %d  Pitch: %d    Yaw: %d", roll, pitch, yaw));
    }

    private void startService(Class<?> service, ServiceConnection serviceConnection, Bundle extras) {


        if(!UsbService.SERVICE_CONNECTED)
        {

            Intent startService = new Intent(this, service);
            if(extras != null && !extras.isEmpty())
            {
                Set<String> keys = extras.keySet();
                for(String key: keys)
                {
                    String extra = extras.getString(key);
                    startService.putExtra(key, extra);
                }
            }
            startService(startService);
        }
        Intent bindingIntent = new Intent(this, service);
        bindService(bindingIntent, serviceConnection, Context.BIND_AUTO_CREATE);
    }

    private void setFilters() {
        IntentFilter filter = new IntentFilter();
        filter.addAction(UsbService.ACTION_USB_PERMISSION_GRANTED);
        filter.addAction(UsbService.ACTION_NO_USB);
        filter.addAction(UsbService.ACTION_USB_DISCONNECTED);
        filter.addAction(UsbService.ACTION_USB_NOT_SUPPORTED);
        filter.addAction(UsbService.ACTION_USB_PERMISSION_NOT_GRANTED);
        registerReceiver(mUsbReceiver, filter);
    }

    private final BroadcastReceiver mUsbReceiver = new BroadcastReceiver() {
        @Override
        public void onReceive(Context arg0, Intent arg1)
        {
            if(arg1.getAction().equals(UsbService.ACTION_USB_PERMISSION_GRANTED)) // USB PERMISSION GRANTED
            {
                Toast.makeText(arg0, "USB Ready", Toast.LENGTH_SHORT).show();
            }else if(arg1.getAction().equals(UsbService.ACTION_USB_PERMISSION_NOT_GRANTED)) // USB PERMISSION NOT GRANTED
            {
                Toast.makeText(arg0, "USB Permission not granted", Toast.LENGTH_SHORT).show();
            }else if(arg1.getAction().equals(UsbService.ACTION_NO_USB)) // NO USB CONNECTED
            {
                Toast.makeText(arg0, "No USB connected", Toast.LENGTH_SHORT).show();
            }else if(arg1.getAction().equals(UsbService.ACTION_USB_DISCONNECTED)) // USB DISCONNECTED
            {
                Toast.makeText(arg0, "USB disconnected", Toast.LENGTH_SHORT).show();
            }else if(arg1.getAction().equals(UsbService.ACTION_USB_NOT_SUPPORTED)) // USB NOT SUPPORTED
            {
                Toast.makeText(arg0, "USB device not supported", Toast.LENGTH_SHORT).show();
            }
        }
    };

    private final ServiceConnection usbConnection = new ServiceConnection() {
        @Override
        public void onServiceConnected(ComponentName arg0, IBinder arg1)
        {
            usbService = ((UsbService.UsbBinder) arg1).getService();
            usbService.setHandler(mHandler);

            // When the connection is made, send the flight controller a request for ATTITUDE messages
            sendAttitudeRequest();
        }

        @Override
        public void onServiceDisconnected(ComponentName arg0)
        {
            usbService = null;
        }
    };


    public static class MyHandler extends Handler  implements ATTITUDE_Handler {

        private MainActivity mMainActivity;

        private Parser parser;

        public static Short[] orientation;
        private static byte[] attitude_request;


        public MyHandler(MainActivity activity) {
            mMainActivity = activity;
            parser    = new Parser();
            parser.set_ATTITUDE_Handler(this);

            orientation = new Short[] {0,0,0};
            attitude_request = parser.serialize_ATTITUDE_Request();
        }

        //Sends the request so long as a usb is plugged in
        public static void sendRequest(byte[] request){
            if (usbService != null){
                usbService.write(request);
            }
        }


        public void handle_ATTITUDE(short roll, short pitch, short yaw) {

            //rollText.setText(String.format("Roll: %d", roll))

            mMainActivity.setAttitudeText(roll, pitch, yaw);

            sendAttitudeRequest();  //sends a request to continue the call and response.
        }


        @Override
        public void handleMessage(Message msg) {

            switch(msg.what)
            {
                case UsbService.MESSAGE_FROM_SERIAL_PORT:

                    byte [] data = (byte []) msg.obj;
                    for (byte b : data) {
                        parser.parse(b);
                    }
                    break;
            }
        }
    }

    //Sends an attitude request if the current tab allows for it.
    public static void sendAttitudeRequest(){

        MyHandler.sendRequest(MyHandler.attitude_request);
    }
}
