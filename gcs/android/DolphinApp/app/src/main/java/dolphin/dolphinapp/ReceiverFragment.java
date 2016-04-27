package dolphin.dolphinapp;

import android.os.AsyncTask;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.ProgressBar;
import android.widget.TextView;
import android.os.Build;
import java.util.ArrayList;

/**
 * creates a receiver fragment
 * Created by Alec Singer on 7/13/15.
 */

public class ReceiverFragment extends Fragment {
    private MainActivity mainActivity;
    private boolean TRUE, isRunning;
    private ArrayList<ProgressBar> bars;
    private ArrayList<TextView> textBoxes;
    private int i;
    private final static int start = 988, end = 2011;       //Start is the minimum value received by the rc receiver and end is the maximum value that could be received.
    private displayMessages display;

    public ReceiverFragment() {
        this.mainActivity = (MainActivity) getActivity();
    }

    public ReceiverFragment(MainActivity mainActivity) {
        this.mainActivity = mainActivity;
    }

    /**
     * calculates the progress out of 100 to display in progressbar.
     */
    public int calculateProgress(int minimum, int maximum, int progress) {
        float newMaximum = maximum - minimum; //Must keep this and the following line separate to keep from rounding
        float constant = 100 / newMaximum;
        return Math.round(constant * (progress - minimum));
    }

    /**
     * sleeps the thread for a given time, @param time.
     */
    public void sleepT(int time) {
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    /**
     * Sets the text of the given TextView, @param textBox, to @param newNumber
     */
    public void setText(TextView textBox, int newNumber) {
        //Sets the text of a given TextView to an integer newNumber
        final TextView textField = textBox;
        final String value = "" + Math.max(newNumber, start);   //ensures that the value displayed is above the minimum start value
        mainActivity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                textField.setText(value);
            }
        });
    }

    /**
     * runs to create the view.  it also initializes all of the class variables and begins the displayMessages asyncTask.
     */
    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {

        View rootView = inflater.inflate(R.layout.fragment_receiver, container, false);
        setRetainInstance(true); // preserves fragment across the teardown/reconstruction cycle when switching orientation
        ProgressBar rollBar, pitchBar, throttleBar, yawBar, switchBar;
        TextView valueRoll, valuePitch, valueThrottle, valueYaw, valueSwitch;

        rollBar = (ProgressBar) rootView.findViewById(R.id.roll_bar);
        pitchBar = (ProgressBar) rootView.findViewById(R.id.pitch_bar);
        throttleBar = (ProgressBar) rootView.findViewById(R.id.throttle_bar);
        yawBar = (ProgressBar) rootView.findViewById(R.id.yaw_bar);
        switchBar = (ProgressBar) rootView.findViewById(R.id.switch_bar);

        valueRoll = (TextView) rootView.findViewById(R.id.rollValue);
        valuePitch = (TextView) rootView.findViewById(R.id.pitchValue);
        valueThrottle = (TextView) rootView.findViewById(R.id.throttleValue);
        valueYaw = (TextView) rootView.findViewById(R.id.yawValue);
        valueSwitch = (TextView) rootView.findViewById(R.id.switchValue);

        mainActivity = (MainActivity) getActivity();

        TRUE = true;

        bars = new ArrayList<ProgressBar>();
        bars.add(rollBar);
        bars.add(pitchBar);
        bars.add(throttleBar);
        bars.add(yawBar);
        bars.add(switchBar);

        textBoxes = new ArrayList<TextView>();
        textBoxes.add(valueRoll);
        textBoxes.add(valuePitch);
        textBoxes.add(valueThrottle);
        textBoxes.add(valueYaw);
        textBoxes.add(valueSwitch);

        MainActivity.setTabBoolean(3, true);

        return rootView;
    }

    /**
     * gets the RC values from the main Activity and puts them in the order Roll, Pitch, Throttle, Yaw, Switch.
     */
    public Short[] getRcValuesFromActivity(){
        Short[] temp = mainActivity.getRcValuesFromHandler();
        return new Short[]{temp[0], temp[1], temp[3], temp[2], temp[4]};
    }

    class displayMessages extends AsyncTask<Void, Short, Void> {

        /**
         * this method runs continuously in the background, and pushes the RC values to onProgressUpdate.
         */
        @Override
        protected Void doInBackground(Void... nothing) {
            isRunning = true;
            while (TRUE) {      //we used the boolean variable "TRUE" for the async task because it should run as long as the tab is being used, but "true" give an error because "doInBackground" would never end.
                if (mainActivity.getNewRC()) {          //checks to see if there is a new message
                    publishProgress(getRcValuesFromActivity());
                    mainActivity.setNewRC(false);       //sets this to false to indicate that the most recent message received has been used
                    sleepT(50);
                }
            }
            return null;
        }

        /**
         * goes through each bar and corresponding text view and updates them according to the current rc values.
         */
        @Override
        protected void onProgressUpdate(Short... values) {
            super.onProgressUpdate(values);
            for (i = 0; i < bars.size(); i++) {
                setText(textBoxes.get(i), values[i]);
                bars.get(i).setProgress(calculateProgress(start, end, values[i]));
            }
        }
    }

    @Override
    public void onResume(){
        super.onResume();
        isRunning = false;      //the async task should not be running at this point in onResume

        display = new displayMessages();

        if (Build.VERSION.SDK_INT>=Build.VERSION_CODES.HONEYCOMB)               //Without this if ReceiverFragment will only work if it is clicked on before MessageFragment
            display.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
        else
            display.execute();

        sleepT(100);                            //gives the app .1 seconds to start the async task before manually forcing it to start.
        if (!isRunning){                                                        //Without this if ReceiverFragment can only be accessed a limited number of times before it stops working or the app must be quit.
            new Thread(new Runnable() {
                public void run() {
                    display.doInBackground();
                }
            }).start();
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        MainActivity.setTabBoolean(3, false);
        display.cancel(true);           //Stops the task from running in the background of other threads
        isRunning = false;              //Shows the task is no longer running
    }

    //Returns the value start
    public static short getStart(){
        return (short) start;
    }
}
