package levy.cs.wlu.edu.hackflight;

import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.content.ServiceConnection;
import android.support.v4.view.ViewPager;
import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.util.Log;
import android.widget.Toast;
import android.content.BroadcastReceiver;
import android.content.ComponentName;
import android.os.IBinder;
import android.os.Handler;
import android.os.Message;

import java.lang.ref.WeakReference;
import java.util.Set;

import edu.wlu.cs.msppg.ATTITUDE_Handler;
import edu.wlu.cs.msppg.Parser;
import edu.wlu.cs.msppg.RC_Handler;

public class MainActivity extends AppCompatActivity {

    private static UsbService usbService;

    private static boolean newAttitude;             //if true: new attitude message available.  if false: most recent attitude message has been processed by the appropriate fragment
    private static boolean newRC;                   //if true: new RC message available.        if false: most recent RC message has been processed by the appropriate fragment
    private static boolean AttitudeRequestBoolean;  //if true: current fragment can access attitude data.      if false: current fragment cannot access attitude data
    private static boolean RcRequestBoolean;        //if true: current fragment can access RC data.            if false: current fragment cannot access RC data

    //if true: ____Fragment view has been created and is running.  if false: ____Fragment view is not active.
    //tabs are considered active by ViewPager if you are in a tab, or directly to the right/left of it.
    private static boolean setupBoolean;
    private static boolean motorBoolean;
    private static boolean mapsBoolean;
    private static boolean receiverBoolean;
    private static boolean messagesBoolean;


    /**
     * The {@link ViewPager} that will host the section contents.
     */
    ViewPager mViewPager;

    private MyHandler mHandler;

    @Override
    protected void onCreate(Bundle savedInstanceState) {

        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //There cannot be a connected usb service as of this point, therefore we cannot send messages.
        setAttitudeRequestBoolean(false);
        setRcRequestBoolean(false);

        startService(UsbService.class, usbConnection, null);        //starts the usb service

        mHandler = new MyHandler(this);
    }

    @Override
    public void onResume() {
        super.onResume();
        setFilters();  // Start listening notifications from UsbService
        startService(UsbService.class, usbConnection, null); // Start UsbService(if it was not started before) and Bind it
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

            adjustMessages();
        }

        @Override
        public void onServiceDisconnected(ComponentName arg0)
        {
            usbService = null;
        }
    };


    public static class MyHandler extends Handler  implements ATTITUDE_Handler, RC_Handler{

        private final WeakReference<MainActivity> mActivity;

        private Parser parser;

        public static Short[] orientation;
        public static Short[] rcValues;
        private static byte[] attitude_request;
        private static byte[] rc_request;


        public MyHandler(MainActivity activity) {
            mActivity = new WeakReference<MainActivity>(activity);
            parser    = new Parser();
            parser.set_ATTITUDE_Handler(this);
            parser.set_RC_Handler(this);

            orientation = new Short[] {0,0,0};

            short start = ReceiverFragment.getStart();
            rcValues    = new Short[] {start, start, start, start, start, start, start, start};

            attitude_request = parser.serialize_ATTITUDE_Request();
            rc_request       = parser.serialize_RC_Request();
        }

        //Sends the request so long as a usb is plugged in
        public static void sendRequest(byte[] request){
            if (usbService != null){
                usbService.write(request);
            }
        }


        public void handle_ATTITUDE(short roll, short pitch, short yaw) {
            Log.d("TAG", "handle_ATTITUDE");
            orientation = new Short[] {roll, pitch, yaw};
            setNewAttitude(true);                           //sets this to true to indicate there is a new message
            sendAttitudeRequest();                          //sends a request to continue the call and response.
        }

        public void handle_RC(short c1, short c2, short c3, short c4, short c5, short c6, short c7, short c8){
            rcValues = new Short[] {c1, c2, c3, c4, c5, c6, c7, c8};
            setNewRC(true);
            sendRCRequest();
        }

        public static String getAttitudeMessage(){
            return "roll = " + addSpacingRoll("" + orientation[0]) + "pitch = " + addSpacingPitch("" + orientation[1]) + "yaw = " + orientation[2];
        }

        private static String addSpacingRoll(String value){
            if (value.length() == 1)
                value = value + "             \t";
            else if (value.length() == 2)
                value = value + "           \t";
            else if (value.length() == 3)
                value = value + "         \t";
            else if (value.length() == 4)
                value = value + "       \t";
            else if (value.length() == 5)
                value = value + "      \t";
            return value;
        }
        private static String addSpacingPitch(String value){
            if (value.length() == 1)
                value = value + "               \t";
            else if (value.length() == 2)
                value = value + "              \t";
            else if (value.length() == 3)
                value = value + "            \t";
            else if (value.length() == 4)
                value = value + "         \t";
            else if (value.length() == 5)
                value = value + "      \t";
            return value;
        }

        public static Short[] getOrientation() {
            return orientation;
        }

        public static Short[] getRcValues() {
            return rcValues;
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

        // XXX remove when app fully implemented
        AttitudeRequestBoolean = true;

        if (AttitudeRequestBoolean) {
            MyHandler.sendRequest(MyHandler.attitude_request);
        }
    }

    //Sends an rc request if the current tab allows for it.
    public static void sendRCRequest(){
        if (RcRequestBoolean) {
            MyHandler.sendRequest(MyHandler.rc_request);
        }
    }

    //Getters and setters for the newAttitude and newRC variables
    public static void setNewAttitude(boolean value){
        newAttitude = value;
    }
    public static boolean getNewAttitude(){
        return newAttitude;
    }

    public static void setNewRC(boolean value){
        newRC = value;
    }
    public static boolean getNewRC(){
        return newRC;
    }

    //setters for AttitudeRequestBoolean and RcRequestBoolean
    private static void setAttitudeRequestBoolean(boolean value){
        AttitudeRequestBoolean = value;
    }
    private static void setRcRequestBoolean(boolean value){
        RcRequestBoolean = value;
    }


    //get the data from the handler so they can be accessed by the fragments.
    public String getAttitudeFromHandler(){
        return MyHandler.getAttitudeMessage();
    }
    public static Short[] getOrientationFromHandler() {
        return MyHandler.getOrientation();
    }
    public static Short[] getRcValuesFromHandler() {
        return MyHandler.getRcValues();
    }



    //the setupFragment seems to leak over to the tab to the right of it. to stop this the view is removed when the setup tab is no longer selected.
    private static void adjustView(){
        if (setupBoolean && motorBoolean && !mapsBoolean){              //in the SETUP tab
            SetupFragment.showView();
        }
        else if (setupBoolean && motorBoolean && mapsBoolean){          //in the MOTOR tab
            SetupFragment.hideView();
        }
    }

    //see green text below to understand the need for this
    public static void setTabBoolean(int tabNumber, boolean value){
        if(tabNumber == 0){
            setupBoolean = value;
        }
        else if(tabNumber == 1){
            motorBoolean = value;
        }
        else if(tabNumber == 2){
            mapsBoolean = value;
        }
        else if(tabNumber == 3){
            receiverBoolean = value;
        }
        else{
            messagesBoolean = value;
        }
        adjustMessages();
        adjustView();
    }

    /**
     * ViewPager creates the views of the tab to the left and right of the tab selected.  To determine if a tab is selected, we look at what views are active.
     * if setup and motor are active, but maps is not then we must be in the Setup tab.  this is because the setup tab would be active because we are on it,
     * and the motor tab is active because it is one tab away from the current tab.  With this information of what tab we are currently on we can tailor the
     * messages allowed to be sent out.  Because the setup tab only requires the attitude to function, AttitudeRequestBoolean is set to true and RcRequestBoolean
     * is set to false.  we then send out an attitude request to start the send and receive cycle.
     */
    public static void adjustMessages(){

        // XXX remove when app fully implemented
        sendAttitudeRequest();

        if (setupBoolean && motorBoolean && !mapsBoolean){              //in the SETUP tab
            setAttitudeRequestBoolean(true);
            setRcRequestBoolean(false);
            sendAttitudeRequest();
        }
        else if (setupBoolean && motorBoolean && mapsBoolean){          //in the MOTOR tab
            setAttitudeRequestBoolean(false);
            setRcRequestBoolean(false);
        }
        else if (motorBoolean && mapsBoolean && receiverBoolean){       //in the MAPS tab
            setAttitudeRequestBoolean(false);
            setRcRequestBoolean(false);
        }
        else if (mapsBoolean && receiverBoolean && messagesBoolean){    //in the RECEIVER tab
            setAttitudeRequestBoolean(false);
            setRcRequestBoolean(true);
            sendRCRequest();
        }
        else if (!mapsBoolean && receiverBoolean && messagesBoolean){   //in the MESSAGES tab
            setAttitudeRequestBoolean(true);
            setRcRequestBoolean(true);
            sendAttitudeRequest();
            sendRCRequest();
        }
    }

}