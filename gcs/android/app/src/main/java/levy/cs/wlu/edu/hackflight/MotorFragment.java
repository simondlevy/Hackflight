package levy.cs.wlu.edu.hackflight;

import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.support.v4.app.FragmentTransaction;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.CheckBox;
import android.widget.CompoundButton;
import android.widget.SeekBar;
import android.widget.TextView;
import android.widget.ToggleButton;

/**
 * creates a fragment that allows for motor setup.
 * Created by Rob Jones and Alec Singer on 7/13/15.
 */
public class MotorFragment extends Fragment {
    /**
     * The fragment argument representing the section number for this
     * fragment.
     */
    private static final String ARG_SECTION_NUMBER = "section_number";
    ToggleButton motor1Toggle, motor2Toggle, motor3Toggle, motor4Toggle;
    CheckBox checkBox;
    SeekBar seekBar;

    public static MotorFragment newInstance(int sectionNumber) {
        MotorFragment fragment = new MotorFragment();
        Bundle args = new Bundle();
        args.putInt(ARG_SECTION_NUMBER, sectionNumber);
        fragment.setArguments(args);

        MainActivity.setTabBoolean(1, true);
        return fragment;
    }

    public MotorFragment() {
    }

    //either enables or disables motors
    private void toggleMotors(boolean value){
        motor1Toggle.setEnabled(value);
        motor2Toggle.setEnabled(value);
        motor3Toggle.setEnabled(value);
        motor4Toggle.setEnabled(value);
    }

    //deselects all the motors
    private void deselectMotors(){
        motor1Toggle.setChecked(false);
        motor2Toggle.setChecked(false);
        motor3Toggle.setChecked(false);
        motor4Toggle.setChecked(false);
    }

    //deselects all motors and then checks the one being used.
    public void turnOffUnusedMotors(ToggleButton toggleButton) {
        deselectMotors();
        toggleButton.setChecked(true);
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container,
                             Bundle savedInstanceState) {

        View rootView = inflater.inflate(R.layout.fragment_motor, container, false);
        super.onCreate(savedInstanceState);

        motor1Toggle = (ToggleButton) rootView.findViewById(R.id.toggleButton1);
        motor2Toggle = (ToggleButton) rootView.findViewById(R.id.toggleButton2);
        motor3Toggle = (ToggleButton) rootView.findViewById(R.id.toggleButton3);
        motor4Toggle = (ToggleButton) rootView.findViewById(R.id.toggleButton4);
        checkBox = (CheckBox) rootView.findViewById(R.id.checkBox);
        seekBar = (SeekBar) rootView.findViewById(R.id.seekBar);
        final TextView seekBarValue = (TextView) rootView.findViewById(R.id.textView);


        seekBar.setEnabled(false);      //default is to disable functionality before user removes all motors
        checkBox.setChecked(false);     //default is to disable functionality before user removes all motors


        //Adds listeners to the buttons
        motor1Toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    turnOffUnusedMotors(motor1Toggle);
                    seekBar.setProgress(0);
                }
            }
        });
        motor2Toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    turnOffUnusedMotors(motor2Toggle);
                    seekBar.setProgress(0);
                }
            }
        });
        motor3Toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    turnOffUnusedMotors(motor3Toggle);
                    seekBar.setProgress(0);
                }
            }
        });
        motor4Toggle.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    turnOffUnusedMotors(motor4Toggle);
                    seekBar.setProgress(0);

                }
            }
        });
        checkBox.setOnCheckedChangeListener(new CompoundButton.OnCheckedChangeListener() {
            @Override
            public void onCheckedChanged(CompoundButton buttonView, boolean isChecked) {
                if (isChecked) {
                    seekBar.setEnabled(true);
                    motor1Toggle.setChecked(true);
                    toggleMotors(true);

                } else {
                    seekBar.setProgress(0);
                    seekBar.setEnabled(false);

                    deselectMotors();   //disables and deselects all motors
                    toggleMotors(false);
                }
            }
        });
        seekBar.setOnSeekBarChangeListener(new SeekBar.OnSeekBarChangeListener() {
            @Override
            public void onProgressChanged(SeekBar seekBar, int progress, boolean fromUser) {
                seekBarValue.setText(String.valueOf(progress));
            }

            @Override
            public void onStartTrackingTouch(SeekBar seekBar) {
            }

            @Override
            public void onStopTrackingTouch(SeekBar seekBar) {
            }
        });
        return rootView;
    }

    @Override
    public void onPause() {
        super.onPause();
        FragmentTransaction ft = getActivity().getSupportFragmentManager().beginTransaction();      //restart the fragment so the slider is set to 0 for the next use.
        ft.remove(this);
        ft.commit();
        MainActivity.setTabBoolean(1, false);
    }
}
