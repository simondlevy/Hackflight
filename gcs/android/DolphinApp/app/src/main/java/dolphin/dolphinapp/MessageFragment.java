package dolphin.dolphinapp;

import android.os.AsyncTask;
import android.os.Build;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.ScrollView;
import android.widget.TextView;
import android.util.Log;

/**
 * Creates a message fragment
 * Message fragment displays messages received from FMU
 * Created by Alec Singer and Rob Jones on 7/13/15.
 */
public class MessageFragment extends Fragment {
    /**
     * The fragment argument representing the section number for this
     * fragment.
     */
    private static final String ARG_SECTION_NUMBER = "section_number";
    private TextView messageText;
    private ScrollView messageScrollView;
    private MainActivity activity;
    protected boolean TRUE = true;
    boolean autoScroll = true;
    private displayMessages display;

    /**
     * Returns a new instance of this fragment for the given section
     * number.
     */
    public static MessageFragment newInstance(int sectionNumber) {
        MessageFragment fragment = new MessageFragment();
        Bundle args = new Bundle();
        args.putInt(ARG_SECTION_NUMBER, sectionNumber);
        fragment.setArguments(args);
        MainActivity.setTabBoolean(4, true);

        return fragment;
    }

    public MessageFragment() {
    }

    public void sleepT(int time) {
        //Sleeps the thread for a given time
        try {
            Thread.sleep(time);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_message, container, false);
        setRetainInstance(true);
        messageText = (TextView) rootView.findViewById(R.id.messageText);
        messageScrollView = (ScrollView) rootView.findViewById(R.id.scrollerId);
        CheckBox messagesCheckBox = (CheckBox) rootView.findViewById(R.id.messagesCheckBox);

        messagesCheckBox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                autoScroll = !isChecked;
            }

        });

        activity = (MainActivity) getActivity();

        MainActivity.setTabBoolean(4, true);

        display = new displayMessages();
        display.execute();

        return rootView;
    }

    /**
     * retrieves FMU messages received in main activity
     * @return message from activity
     */
    private String getAttitudeFromActivity(){
        return activity.getAttitudeFromHandler();
    }
    /**
     * retrieves RC messages received in main activity
     */
    private String getRCFromActivity(){
        Short[] temp = activity.getRcValuesFromHandler();
        return "c1=" + addSpacing(temp[0]) + "c2=" + addSpacing(temp[1]) + "c3=" + addSpacing(temp[2]) + "c4=" + addSpacing(temp[3]) + "c5=" + temp[4];
    }

    private static String addSpacing(Short input){
        String value = String.valueOf(input);
        if (value.length() == 3)
            value = value + "    \t";
        else if (value.length() == 4)
            value = value + " \t";
        return value;
    }

    /**
     * Async task that runs in OnCreateView. updates the textView to display messages from FMU.
     */
    class displayMessages extends AsyncTask<Void, String, Void> {

        @Override
        protected Void doInBackground(Void... nothing) {
            while (TRUE) {
                if (activity.getNewAttitude()){             //checks to see if there is a new attitude message
                    publishProgress(getAttitudeFromActivity());
                    activity.setNewAttitude(false);         //sets this to false to indicate that the most recent attitude message received has been used
                }
                if (activity.getNewRC()){                   //checks to see if there is a new rc message
                    publishProgress(getRCFromActivity());
                    activity.setNewRC(false);               //sets this to false to indicate that the most recent rc message received has been used
                }
                sleepT(50);
            }
            return null;
        }

        @Override
        protected void onProgressUpdate(String... string) {
            super.onProgressUpdate(string);
            messageText.append(string[0] + '\n');

            if(autoScroll) {                    //scrolls to the bottom of the screen if the check box is  unchecked
                scrollToBottom(messageScrollView);
            }
        }

        /**
         * Thread that scrolls to bottom of scrollview as new messages are added
         */
        protected void scrollToBottom(ScrollView scrollView){
            final ScrollView finalScrollView = scrollView;      //converted to a final to avoid an error
            if (finalScrollView != null){
                finalScrollView.post(new Runnable() {
                    @Override
                    public void run() {
                        finalScrollView.fullScroll(ScrollView.FOCUS_DOWN);
                    }
                });
            }
        }
    }

    @Override
    public void onResume(){
        super.onResume();
        display = new displayMessages();
        if (Build.VERSION.SDK_INT>=Build.VERSION_CODES.HONEYCOMB)       //Without this if MessageFragment will only work if it is clicked on before ReceiverFragment
            display.executeOnExecutor(AsyncTask.THREAD_POOL_EXECUTOR);
        else
            display.execute();
    }

    @Override
    public void onPause(){
        super.onPause();
        MainActivity.setTabBoolean(4, false);
        display.cancel(true);       //Stops the task from running in the background of other threads
    }

}
