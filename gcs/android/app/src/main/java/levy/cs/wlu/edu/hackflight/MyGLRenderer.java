package levy.cs.wlu.edu.hackflight;

import android.opengl.GLSurfaceView;
import android.opengl.GLU;

import java.lang.Override;
import javax.microedition.khronos.egl.EGLConfig;
import javax.microedition.khronos.opengles.GL10;

/**
 * Created by Rob Jones on 6/22/15.
 * creates a renderer
 * tutorial link: https://www3.ntu.edu.sg/home/ehchua/programming/android/Android_3D.html
 */
public class MyGLRenderer implements GLSurfaceView.Renderer {

    Quad quad;

    // For controlling quad's z-position, x and y angles
    float angleX = 0;
    float angleY = 0;
    float angleZ = 0;
    float xRatio = 0.1f;
    float zRatio = 0.1f;
    float yRatio = 1.0f;
    float z      = -4.0f;

    public MyGLRenderer() { quad = new Quad(); }

    @Override
    public void onSurfaceCreated(GL10 gl, EGLConfig config) {
        gl.glClearColor(0.0f, 0.0f, 0.0f, 1.0f);  // Set color's clear-value to black
        gl.glClearDepthf(1.0f);            // Set depth's clear-value to farthest
        gl.glEnable(GL10.GL_DEPTH_TEST);   // Enables depth-buffer for hidden surface removal
        gl.glDepthFunc(GL10.GL_LEQUAL);    // The type of depth testing to do
        gl.glHint(GL10.GL_PERSPECTIVE_CORRECTION_HINT, GL10.GL_NICEST);  // nice perspective view
        gl.glShadeModel(GL10.GL_SMOOTH);   // Enable smooth shading of color
        gl.glDisable(GL10.GL_DITHER);      // Disable dithering for better performance
    }

    // Call back after onSurfaceCreated() or whenever the window's size changes
    @Override
    public void onSurfaceChanged(GL10 gl, int width, int height) {
        if (height == 0) height = 1;   // To prevent divide by zero
        float aspect = (float) width / height;

        // Set the viewport (display area) to cover the entire window
        gl.glViewport(0, 0, width, height);

        // Setup perspective projection, with aspect ratio matches viewport
        gl.glMatrixMode(GL10.GL_PROJECTION); // Select projection matrix
        gl.glLoadIdentity();                 // Reset projection matrix
        // Use perspective projection
        GLU.gluPerspective(gl, 45, aspect, 0.1f, 100.f);

        gl.glMatrixMode(GL10.GL_MODELVIEW);  // Select model-view matrix
        gl.glLoadIdentity();                 // Reset
    }

    private static Short[] getOrientationFromActivity(){
        return MainActivity.getOrientationFromHandler();
    }

    // Call back to draw the current frame.
    @Override
    public void onDrawFrame(GL10 gl) {
        // Clear color and depth buffers using clear-value set earlier
        gl.glClear(GL10.GL_COLOR_BUFFER_BIT | GL10.GL_DEPTH_BUFFER_BIT);

        // - - - - Render the Quad - - - -
        gl.glLoadIdentity();
        gl.glTranslatef(0.0f, 0.0f, z);
        gl.glRotatef(angleY, 0.0f, 1.0f, 0.0f);
        gl.glRotatef(angleZ, 0.0f, 0.0f, 1.0f);//Rotate
        gl.glRotatef(angleX, 1.0f, 0.0f, 0.0f);
        quad.draw(gl);
        // Update the rotational angle after each refresh
        angleX = -((getOrientationFromActivity()[1] + 900) * xRatio) + 90;
        angleY = -getOrientationFromActivity()[2] * yRatio;
        angleZ = -((getOrientationFromActivity()[0] + 1800) * zRatio) + 180;
    }
}
