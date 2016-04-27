package dolphin.dolphinapp;

import android.opengl.GLSurfaceView;
import android.os.Bundle;
import android.support.v4.app.Fragment;
import android.view.LayoutInflater;
import android.view.View;
import android.view.ViewGroup;
import android.widget.FrameLayout;

/**
 * creates a setup fragment
 * Created by Rob Jones on 7/13/15.
 */
public class SetupFragment extends Fragment {
    /**
     * The fragment argument representing the section number for this
     * fragment.
     */
    private static final String ARG_SECTION_NUMBER = "section_number";
    private static GLSurfaceView glView;

    /**
     * Returns a new instance of this fragment for the given section
     * number.
     */
    public static SetupFragment newInstance(int sectionNumber) {
        SetupFragment fragment = new SetupFragment();
        Bundle args = new Bundle();
        args.putInt(ARG_SECTION_NUMBER, sectionNumber);
        fragment.setArguments(args);

        MainActivity.setTabBoolean(0, true);

        return fragment;
    }

    public SetupFragment() {
    }

    @Override
    public View onCreateView(LayoutInflater inflater, ViewGroup container, Bundle savedInstanceState) {
        View rootView = inflater.inflate(R.layout.fragment_setup, container, false);
        super.onCreate(savedInstanceState);
        glView = new MyGLSurfaceView(this.getActivity());
        FrameLayout frame = (FrameLayout) rootView.findViewById(R.id.myFrameLayout);
        frame.addView(glView);
        return rootView;
    }

    public static void hideView() {
        if (glView != null){
            glView.setVisibility(glView.GONE);
        }
    }

    public static void showView() {
        if (glView != null){
            glView.setVisibility(glView.VISIBLE);
        }
    }

    @Override
    public void onPause() {
        super.onPause();
        MainActivity.setTabBoolean(0, false);
    }

}











