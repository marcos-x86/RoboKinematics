/**
 *RoboKinematics is a basic application for students in electronics,
 *mechanics and mechatronics. This application allows to calculate
 *the forward kinematics and the inverse kinematics of industrial
 *robots with rotational joints, up to 3 degrees of freedom (RRR).
 */

package com.prometheus.marcos.robokinematics;

import android.support.v7.app.AppCompatActivity;
import android.os.Bundle;
import android.view.View;
import android.widget.Button;
import android.widget.CheckBox;
import android.widget.EditText;
import android.widget.ImageView;
import android.widget.ScrollView;
import android.widget.TextView;

public class MainActivity extends AppCompatActivity {

    ScrollView myScrollView;

    Button button_setup;
    Button button_fk;
    Button button_ik;

    CheckBox check_1dof;
    CheckBox check_2dof;
    CheckBox check_3dof;

    ImageView image_sol;
    ImageView image_loj;
    ImageView image_fk;
    ImageView image_ik;

    EditText editText_loj1;
    EditText editText_loj2;
    EditText editText_loj3;
    EditText editText_loj1_1;
    EditText editText_loj2_2;
    EditText editText_loj3_3;
    EditText editText_sof1;
    EditText editText_sof2;
    EditText editText_sof3;
    EditText editText_fk_alpha;
    EditText editText_fk_beta;
    EditText editText_fk_theta;
    EditText editText_ik_x;
    EditText editText_ik_y;
    EditText editText_ik_z;

    TextView textView_fk_resX;
    TextView textView_fk_resY;
    TextView textView_fk_resZ;
    TextView textView_ik_resAlpha;
    TextView textView_ik_resBeta;
    TextView textView_ik_resTheta;
    TextView textView_message;
    TextView textView_messageFk;
    TextView textView_messageIk;

    int config;

    /**
     *This override  method initialize all variables that will contain
     *the views used (EditText, TextView, Button, ScrollView, ImageView
     *and Checkbox).
     */
    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        myScrollView = (ScrollView) findViewById(R.id.scroll_main);

        button_setup = (Button) findViewById(R.id.button_setup);
        button_fk = (Button) findViewById(R.id.button_fk);
        button_ik = (Button) findViewById(R.id.button_ik);

        check_1dof = (CheckBox) findViewById(R.id.check_1dof);
        check_2dof = (CheckBox) findViewById(R.id.check_2dof);
        check_3dof = (CheckBox) findViewById(R.id.check_3dof);

        image_sol = (ImageView) findViewById(R.id.image_sol);
        image_loj = (ImageView) findViewById(R.id.image_loj);
        image_fk = (ImageView) findViewById(R.id.image_fk);
        image_ik = (ImageView) findViewById(R.id.image_ik);

        editText_loj1 = (EditText) findViewById(R.id.editText_loj1);
        editText_loj2 = (EditText) findViewById(R.id.editText_loj2);
        editText_loj3 = (EditText) findViewById(R.id.editText_loj3);
        editText_loj1_1 = (EditText) findViewById(R.id.editText_loj1_1);
        editText_loj2_2 = (EditText) findViewById(R.id.editText_loj2_2);
        editText_loj3_3 = (EditText) findViewById(R.id.editText_loj3_3);
        editText_sof1 = (EditText) findViewById(R.id.editText_sof1);
        editText_sof2 = (EditText) findViewById(R.id.editText_sof2);
        editText_sof3 = (EditText) findViewById(R.id.editText_sof3);
        editText_fk_alpha = (EditText) findViewById(R.id.editText_fk_alpha);
        editText_fk_beta = (EditText) findViewById(R.id.editText_fk_beta);
        editText_fk_theta = (EditText) findViewById(R.id.editText_fk_theta);
        editText_ik_x = (EditText) findViewById(R.id.editText_ik_x);
        editText_ik_y = (EditText) findViewById(R.id.editText_ik_y);
        editText_ik_z = (EditText) findViewById(R.id.editText_ik_z);

        textView_fk_resX = (TextView) findViewById(R.id.textView_fk_resX);
        textView_fk_resY = (TextView) findViewById(R.id.textView_fk_resY);
        textView_fk_resZ = (TextView) findViewById(R.id.textView_fk_resZ);
        textView_ik_resAlpha = (TextView) findViewById(R.id.textView_ik_resAlpha);
        textView_ik_resBeta = (TextView) findViewById(R.id.textView_ik_resBeta);
        textView_ik_resTheta = (TextView) findViewById(R.id.textView_ik_resTheta);
        textView_message = (TextView) findViewById(R.id.textView_message);
        textView_messageFk = (TextView) findViewById(R.id.textView_messageFk);
        textView_messageIk = (TextView) findViewById(R.id.textView_messageIk);

        setInitialConf();

    }

    /**
     *This method is responsible for setting all configurations Robot.
     *Evaluate whether the parameters entered by the user are valid or
     *if any is needed is missing. This in turn calls other methods
     *that help in this task.
     */

    public void setRobotValues(View view) {
        if ((button_setup.getText().toString().matches("SET ALL ROBOT CONFIGURATIONS"))) {
            if (check_1dof.isChecked()) {
                if (verifyLojOneDof()) {
                    button_setup.setText("RE-CONFIG ROBOT");
                    config = 1;
                    enableKinematicsOneDof();
                } else {
                    textView_message.setText("Data input error. Text fields empty or Limit of joints out of range");
                }
            } else if (check_2dof.isChecked()) {
                if (verifyLojTwoDof()) {
                    button_setup.setText("RE-CONFIG ROBOT");
                    config = 2;
                    enableKinematicsTwoDof();
                } else {
                    textView_message.setText("Data input error. Text fields empty or Limit of joints out of range");
                }
            } else if (check_3dof.isChecked()) {
                if (verifyLojThreeDof()) {
                    button_setup.setText("RE-CONFIG ROBOT");
                    config = 3;
                    enableKinematicsThreeDof();
                } else {
                    textView_message.setText("Data input error. Text fields empty or Limit of joints out of range");
                }
            }
        } else {
            if (config == 1) {
                disableKinematics();
                setOneDofConfig();
            } else {
                if (config == 2) {
                    disableKinematics();
                    setTwoDofConfig();
                } else {
                    disableKinematics();
                    setThreeDofConfig();
                }
            }

        }
    }

    /**
     *This method is executed when a click on the button to calculate
     *the fordward Kinematics. There are many methods to calculate the
     *fordward kinematics, the App uses equations determined by the
     *geometrical method for solutions 1 and 2 degrees of freedom and
     *also uses the method of the DH matrix for the case of 3 degrees
     *of freedom. This method is also responsible for evaluating the
     *parameters entered are correct.
     */
    public void forwardKinematics(View view) {
        if (config == 1) {
            if (!verifyFkOneDofEmpty()) {
                double alpha = Double.parseDouble(editText_fk_alpha.getText().toString());
                double link1 = Double.parseDouble(editText_sof1.getText().toString());
                double x = Math.round(link1 * (Math.cos(Math.toRadians(alpha))) * 100.0) / 100.0;
                double y = Math.round(link1 * (Math.sin(Math.toRadians(alpha))) * 100.0) / 100.0;
                textView_fk_resX.setText("" + x);
                textView_fk_resY.setText("" + y);
            }
        } else if (config == 2) {
            if (!verifyFkTwoDofEmpty()) {
                double alpha = Double.parseDouble(editText_fk_alpha.getText().toString());
                double beta = Double.parseDouble(editText_fk_beta.getText().toString());
                double link1 = Double.parseDouble(editText_sof1.getText().toString());
                double link2 = Double.parseDouble(editText_sof2.getText().toString());
                double x = Math.round((link1 * Math.cos(Math.toRadians(alpha)) + link2 * Math.cos(Math.toRadians(alpha + beta))) * 100.0) / 100.0;
                double y = Math.round((link1 * Math.sin(Math.toRadians(alpha)) + link2 * Math.sin(Math.toRadians(alpha + beta))) * 100.0) / 100.0;
                textView_fk_resX.setText("" + x);
                textView_fk_resY.setText("" + y);
            }
        } else {
            if (!verifyFkThreeDofEmpty()) {
                double alpha = Double.parseDouble(editText_fk_alpha.getText().toString());
                double beta = Double.parseDouble(editText_fk_beta.getText().toString());
                double theta = Double.parseDouble(editText_fk_theta.getText().toString());
                double link1 = Double.parseDouble(editText_sof1.getText().toString());
                double link2 = Double.parseDouble(editText_sof2.getText().toString());
                double link3 = Double.parseDouble(editText_sof3.getText().toString());
                double[][] matrixA = {{Math.cos(Math.toRadians(alpha)), 0, Math.sin(Math.toRadians(alpha)), 0}, {Math.sin(Math.toRadians(alpha)), 0, -Math.cos(Math.toRadians(alpha)), 0}, {0, 1, 0, link1}, {0, 0, 0, 1}};
                double[][] matrixB = {{Math.cos(Math.toRadians(beta)), -Math.sin(Math.toRadians(beta)), 0, link2 * Math.cos(Math.toRadians(beta))}, {Math.sin(Math.toRadians(beta)), Math.cos(Math.toRadians(beta)), 0, link2 * Math.sin(Math.toRadians(beta))}, {0, 0, 1, 0}, {0, 0, 0, 1}};
                double[][] matrixC = {{Math.cos(Math.toRadians(theta)), -Math.sin(Math.toRadians(theta)), 0, link3 * Math.cos(Math.toRadians(theta))}, {Math.sin(Math.toRadians(theta)), Math.cos(Math.toRadians(theta)), 0, link3 * Math.sin(Math.toRadians(theta))}, {0, 0, 1, 0}, {0, 0, 0, 1}};
                double[][] matrixAux = multiplyMatrix(matrixA, matrixB);
                double[][] matrixDH = multiplyMatrix(matrixAux, matrixC);
                double x = Math.round(100.0 * matrixDH[0][3]) / 100.0;
                double y = Math.round(100.0 * matrixDH[1][3]) / 100.0;
                double z = Math.round(100.0 * matrixDH[2][3]) / 100.0;
                textView_fk_resX.setText("" + x);
                textView_fk_resY.setText("" + y);
                textView_fk_resZ.setText("" + z);
            }
        }
    }

    /**
     *This method computes the inverse kinematics of the robot through the
     *equations established by the geometrical method. Also performs control
     *parameters and rounds results.
     */
    public void inverseKinematics(View view) {
        if (config == 1) {
            if (!verifyIkOneDofEmpty()) {
                double x = Double.parseDouble(editText_ik_x.getText().toString());
                double y = Double.parseDouble(editText_ik_y.getText().toString());
                double alpha = Math.round(Math.toDegrees(Math.atan2(y, x)) * 100.0) / 100.0;
                textView_ik_resAlpha.setText("" + alpha);
            }
        } else if (config == 2) {
            if (!verifyIkTwoDofEmpty()) {
                double x = Double.parseDouble(editText_ik_x.getText().toString());
                double y = Double.parseDouble(editText_ik_y.getText().toString());
                double link1 = Double.parseDouble(editText_sof1.getText().toString());
                double link2 = Double.parseDouble(editText_sof2.getText().toString());
                double hyp = Math.sqrt((Math.pow(x, 2)) + (Math.pow(y, 2)));
                double a = Math.atan2(y, x);
                double b = Math.acos((Math.pow(link1, 2) - Math.pow(link2, 2) + Math.pow(hyp, 2)) / (2 * link1 * link2));
                double c = Math.acos((Math.pow(link1, 2) + Math.pow(link2, 2) - Math.pow(hyp, 2)) / (2 * link1 * link2));
                double alpha = Math.round(Math.toDegrees(a + b) * 100.0) / 100.0;
                double beta = Math.round((Math.toDegrees(c) - 180) * 100.0) / 100.0;
                textView_ik_resAlpha.setText("" + alpha);
                textView_ik_resBeta.setText("" + beta);
            }
        } else {
            if (!verifyIkThreeDofEmpty()) {
                double x = Double.parseDouble(editText_ik_x.getText().toString());
                double y = Double.parseDouble(editText_ik_y.getText().toString());
                double z = Double.parseDouble(editText_ik_z.getText().toString());
                double link1 = Double.parseDouble(editText_sof1.getText().toString());
                double link2 = Double.parseDouble(editText_sof2.getText().toString());
                double link3 = Double.parseDouble(editText_sof3.getText().toString());
                double r1 = Math.sqrt((Math.pow(x, 2)) + (Math.pow(y, 2)));
                double r2 = z - link1;
                double r3 = Math.sqrt((Math.pow(r1, 2)) + (Math.pow(r2, 2)));
                double aux_gamma = Math.atan2(r2, r1);
                double aux_alpha = Math.acos((Math.pow(link2, 2) + Math.pow(link3, 2) - Math.pow(r3, 2)) / (2 * link2 * link3));
                double aux_beta = Math.asin(link3 * (Math.sin(aux_alpha) / r3));
                aux_alpha = Math.toDegrees(aux_alpha);
                aux_beta = Math.toDegrees(aux_beta);
                aux_gamma = Math.toDegrees(aux_gamma);
                double alpha = Math.round(Math.toDegrees(Math.atan2(y, x)) * 100.0) / 100.0;
                double beta = Math.round((aux_gamma + aux_beta) * 100.0) / 100.0;
                double theta = Math.round((-1 * aux_alpha - 180) * 100.0) / 100.0;
                textView_ik_resAlpha.setText("" + alpha);
                textView_ik_resBeta.setText("" + beta);
                textView_ik_resTheta.setText("" + theta);
            }
        }
    }

    /**
     * This method makes the Scrollview is positioned in its default
     *
     */
    public void scrollToTop(View view) {
        myScrollView.scrollTo(0, 0);
    }

    /**
     *This method is called when a click on the button "Clear All",
     *this in turn calls a method that is reused to initialize and to
     *blank out all views.
     *
     */
    public void clearAll(View view) {
        clearAllValues();
    }

    public void clearAllValues() {
        check_1dof.setChecked(false);
        check_2dof.setChecked(false);
        check_3dof.setChecked(false);
        image_fk.setImageResource(R.drawable.fk_blank);
        image_ik.setImageResource(R.drawable.ik_blank);
        image_loj.setImageResource(R.drawable.loj_blank);
        image_sol.setImageResource(R.drawable.sol_blank);
        editText_fk_alpha.setText(null);
        editText_fk_beta.setText(null);
        editText_fk_theta.setText(null);
        editText_ik_x.setText(null);
        editText_ik_y.setText(null);
        editText_ik_z.setText(null);
        editText_loj1.setText(null);
        editText_loj2.setText(null);
        editText_loj3.setText(null);
        editText_loj1_1.setText(null);
        editText_loj2_2.setText(null);
        editText_loj3_3.setText(null);
        editText_sof1.setText(null);
        editText_sof2.setText(null);
        editText_sof3.setText(null);
        textView_fk_resX.setText(null);
        textView_fk_resY.setText(null);
        textView_fk_resZ.setText(null);
        textView_ik_resAlpha.setText(null);
        textView_ik_resBeta.setText(null);
        textView_ik_resTheta.setText(null);
        disableSetup();
        disableKinematics();
        button_setup.setText("SET ALL ROBOT CONFIGURATIONS");
        button_setup.setEnabled(false);
        check_1dof.setEnabled(true);
        check_2dof.setEnabled(true);
        check_3dof.setEnabled(true);

    }

    public void defaultValues(View view) {
        setInitialConf();
    }

    /**
     * Set all views to work the math with a degree of freedom.
     */
    public void setOneDofConfig() {
        image_sol.setImageResource(R.drawable.sol_r);
        image_loj.setImageResource(R.drawable.loj_r);
        image_fk.setImageResource(R.drawable.fk_r);
        image_ik.setImageResource(R.drawable.ik_r);
        editText_loj1.setEnabled(true);
        editText_loj2.setEnabled(false);
        editText_loj3.setEnabled(false);
        editText_loj1_1.setEnabled(true);
        editText_loj2_2.setEnabled(false);
        editText_loj3_3.setEnabled(false);
        editText_sof1.setEnabled(true);
        editText_sof2.setEnabled(false);
        editText_sof3.setEnabled(false);
        editText_loj1.setText("0");
        editText_loj1_1.setText("180");
        button_setup.setEnabled(true);
        button_setup.setText("SET ALL ROBOT CONFIGURATIONS");
        check_1dof.setEnabled(true);
        check_2dof.setEnabled(true);
        check_3dof.setEnabled(true);
        disableKinematics();
    }

    /**
     * Configures all the views to work with two degrees of freedom
     * in the calculations
     */
    public void setTwoDofConfig() {
        image_sol.setImageResource(R.drawable.sol_rr);
        image_loj.setImageResource(R.drawable.loj_rr);
        image_fk.setImageResource(R.drawable.fk_rr);
        image_ik.setImageResource(R.drawable.ik_rr);
        editText_loj1.setEnabled(true);
        editText_loj2.setEnabled(true);
        editText_loj3.setEnabled(false);
        editText_loj1_1.setEnabled(true);
        editText_loj2_2.setEnabled(true);
        editText_loj3_3.setEnabled(false);
        editText_sof1.setEnabled(true);
        editText_sof2.setEnabled(true);
        editText_sof3.setEnabled(false);
        editText_loj1.setText("0");
        editText_loj1_1.setText("180");
        editText_loj2.setText("-90");
        editText_loj2_2.setText("90");
        button_setup.setEnabled(true);
        button_setup.setText("SET ALL ROBOT CONFIGURATIONS");
        check_1dof.setEnabled(true);
        check_2dof.setEnabled(true);
        check_3dof.setEnabled(true);
        disableKinematics();
    }

    /**
     * Configure all views for working with three degrees of freedom
     */
    public void setThreeDofConfig() {
        image_sol.setImageResource(R.drawable.sol_rrr);
        image_loj.setImageResource(R.drawable.loj_rrr);
        image_fk.setImageResource(R.drawable.fk_rrr);
        image_ik.setImageResource(R.drawable.ik_rrr);
        editText_loj1.setEnabled(true);
        editText_loj2.setEnabled(true);
        editText_loj3.setEnabled(true);
        editText_loj1_1.setEnabled(true);
        editText_loj2_2.setEnabled(true);
        editText_loj3_3.setEnabled(true);
        editText_sof1.setEnabled(true);
        editText_sof2.setEnabled(true);
        editText_sof3.setEnabled(true);
        editText_loj1.setText("-180");
        editText_loj1_1.setText("180");
        editText_loj2.setText("0");
        editText_loj2_2.setText("90");
        editText_loj3.setText("-90");
        editText_loj3_3.setText("90");
        button_setup.setEnabled(true);
        button_setup.setText("SET ALL ROBOT CONFIGURATIONS");
        check_1dof.setEnabled(true);
        check_2dof.setEnabled(true);
        check_3dof.setEnabled(true);
        disableKinematics();
    }

    /**
     * Enable controls for forward kinematics with a degree of freedom.
     */
    public void enableKinematicsOneDof() {
        editText_fk_alpha.setEnabled(true);
        editText_fk_beta.setEnabled(false);
        editText_fk_theta.setEnabled(false);
        editText_ik_x.setEnabled(false);
        editText_ik_y.setEnabled(false);
        editText_ik_z.setEnabled(false);
        button_fk.setEnabled(true);
        button_ik.setEnabled(false);
        disableSetup();
    }

    /**
     * Enable controls for forward kinematics with two degrees of freedom.
     */
    public void enableKinematicsTwoDof() {
        editText_fk_alpha.setEnabled(true);
        editText_fk_beta.setEnabled(true);
        editText_fk_theta.setEnabled(false);
        editText_ik_x.setEnabled(true);
        editText_ik_y.setEnabled(true);
        editText_ik_z.setEnabled(false);
        button_fk.setEnabled(true);
        button_ik.setEnabled(true);
        disableSetup();
    }

    /**
     * Enable controls for forward kinematics with three degrees of freedom.
     */
    public void enableKinematicsThreeDof() {
        editText_fk_alpha.setEnabled(true);
        editText_fk_beta.setEnabled(true);
        editText_fk_theta.setEnabled(true);
        editText_ik_x.setEnabled(true);
        editText_ik_y.setEnabled(true);
        editText_ik_z.setEnabled(true);
        button_fk.setEnabled(true);
        button_ik.setEnabled(true);
        disableSetup();
    }

    /**
     * Disable the All Setup Robot controls
     */
    public void disableSetup() {
        check_1dof.setEnabled(false);
        check_2dof.setEnabled(false);
        check_3dof.setEnabled(false);
        editText_loj1.setEnabled(false);
        editText_loj2.setEnabled(false);
        editText_loj3.setEnabled(false);
        editText_loj1_1.setEnabled(false);
        editText_loj2_2.setEnabled(false);
        editText_loj3_3.setEnabled(false);
        editText_sof1.setEnabled(false);
        editText_sof2.setEnabled(false);
        editText_sof3.setEnabled(false);
        button_setup.setText("RE-CONFIG ROBOT");
    }

    /**
     * Disable the All Kinematics controls
     */
    public void disableKinematics() {
        editText_fk_alpha.setEnabled(false);
        editText_fk_beta.setEnabled(false);
        editText_fk_theta.setEnabled(false);
        editText_ik_x.setEnabled(false);
        editText_ik_y.setEnabled(false);
        editText_ik_z.setEnabled(false);
        button_fk.setEnabled(false);
        button_ik.setEnabled(false);
    }

    /**
     * Verify the limit of the joints in the case
     * of work with one degree of freedom mode
     */
    public boolean verifyLojOneDof() {
        boolean res = false;
        if (!verifySetupOneDofEmpty()) {
            int loj1 = Integer.parseInt(editText_loj1.getText().toString());
            int loj1_1 = Integer.parseInt(editText_loj1_1.getText().toString());
            if (loj1 >= 0 && loj1_1 <= 180) {
                res = true;
            } else {
                res = false;
            }
        }
        return res;
    }

    /**
     * Verify the limit of the joints in the case
     * of work with two degrees of freedom mode
     */
    public boolean verifyLojTwoDof() {
        boolean res = false;
        if (!verifySetupTwoDofEmpty()) {
            int loj1 = Integer.parseInt(editText_loj1.getText().toString());
            int loj1_1 = Integer.parseInt(editText_loj1_1.getText().toString());
            int loj2 = Integer.parseInt(editText_loj2.getText().toString());
            int loj2_2 = Integer.parseInt(editText_loj2_2.getText().toString());
            if (loj1 >= 0 && loj1_1 <= 180) {
                if (loj2 >= -90 && loj2_2 <= 90) {
                    return true;
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }
        return res;
    }

    /**
     * Verify the limit of the joints in the case
     * of work with three degrees of freedom mode
     */
    public boolean verifyLojThreeDof() {
        boolean res = false;
        if (!verifySetupThreeDofEmpty()) {
            int loj1 = Integer.parseInt(editText_loj1.getText().toString());
            int loj1_1 = Integer.parseInt(editText_loj1_1.getText().toString());
            int loj2 = Integer.parseInt(editText_loj2.getText().toString());
            int loj2_2 = Integer.parseInt(editText_loj2_2.getText().toString());
            int loj3 = Integer.parseInt(editText_loj3.getText().toString());
            int loj3_3 = Integer.parseInt(editText_loj3_3.getText().toString());
            if (loj1 >= -180 && loj1_1 <= 180) {
                if (loj2 >= 0 && loj2_2 <= 90) {
                    if (loj3 >= -90 && loj3_3 <= 90) {
                        return true;
                    } else {
                        return false;
                    }
                } else {
                    return false;
                }
            } else {
                return false;
            }
        }
        return res;
    }

    /**
     * Verify if the Setup parameters are empty in the case
     * of work with one degree of freedom mode
     */
    public boolean verifySetupOneDofEmpty() {
        String loj1 = editText_loj1.getText().toString();
        String loj1_1 = editText_loj1_1.getText().toString();
        String sof1 = editText_sof1.getText().toString();
        if (loj1.isEmpty() || loj1_1.isEmpty() || sof1.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the Setup parameters are empty in the case
     * of work with two degrees of freedom mode
     */
    public boolean verifySetupTwoDofEmpty() {
        String loj1 = editText_loj1.getText().toString();
        String loj2 = editText_loj2.getText().toString();
        String loj1_1 = editText_loj1_1.getText().toString();
        String loj2_2 = editText_loj2_2.getText().toString();
        String sof1 = editText_sof1.getText().toString();
        String sof2 = editText_sof2.getText().toString();
        if (loj1.isEmpty() || loj2.isEmpty() || loj1_1.isEmpty() || loj2_2.isEmpty() || sof1.isEmpty() || sof2.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the Setup parameters are empty in the case
     * of work with three degrees of freedom mode
     */
    public boolean verifySetupThreeDofEmpty() {
        String loj1 = editText_loj1.getText().toString();
        String loj2 = editText_loj2.getText().toString();
        String loj3 = editText_loj3.getText().toString();
        String loj1_1 = editText_loj1_1.getText().toString();
        String loj2_2 = editText_loj2_2.getText().toString();
        String loj3_3 = editText_loj3_3.getText().toString();
        String sof1 = editText_sof1.getText().toString();
        String sof2 = editText_sof2.getText().toString();
        String sof3 = editText_sof3.getText().toString();
        if (loj1.isEmpty() || loj2.isEmpty() || loj3.isEmpty() || loj1_1.isEmpty() || loj2_2.isEmpty() || loj3_3.isEmpty() || sof1.isEmpty() || sof2.isEmpty() || sof3.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the kinematics parameters are empty
     * in the case of forward kinematics with one degree of freedom
     */
    public boolean verifyFkOneDofEmpty() {
        String fk_alpha = editText_fk_alpha.getText().toString();
        if (fk_alpha.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the kinematics parameters are empty
     * in the case of inverse kinematics with one degree of freedom
     */
    public boolean verifyIkOneDofEmpty() {
        String ik_x = editText_ik_x.getText().toString();
        String ik_y = editText_ik_y.getText().toString();
        if (ik_x.isEmpty() || ik_y.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the kinematics parameters are empty
     * in the case of forward kinematics with two degrees of freedom
     */
    public boolean verifyFkTwoDofEmpty() {
        String fk_alpha = editText_fk_alpha.getText().toString();
        String fk_beta = editText_fk_beta.getText().toString();
        if (fk_alpha.isEmpty() || fk_beta.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the kinematics parameters are empty
     * in the case of inverse kinematics with two degrees of freedom
     */
    public boolean verifyIkTwoDofEmpty() {
        String ik_x = editText_ik_x.getText().toString();
        String ik_y = editText_ik_y.getText().toString();
        if (ik_x.isEmpty() || ik_y.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the kinematics parameters are empty
     * in the case of forward kinematics with three degrees of freedom
     */
    public boolean verifyFkThreeDofEmpty() {
        String fk_alpha = editText_fk_alpha.getText().toString();
        String fk_beta = editText_fk_beta.getText().toString();
        String fk_theta = editText_fk_theta.getText().toString();
        if (fk_alpha.isEmpty() || fk_beta.isEmpty() || fk_theta.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Verify if the kinematics parameters are empty
     * in the case of inverse kinematics with three degrees of freedom
     */
    public boolean verifyIkThreeDofEmpty() {
        String ik_x = editText_ik_x.getText().toString();
        String ik_y = editText_ik_y.getText().toString();
        String ik_z = editText_ik_z.getText().toString();
        if (ik_x.isEmpty() || ik_y.isEmpty() || ik_z.isEmpty()) {
            return true;
        } else {
            return false;
        }
    }

    /**
     * Set the default configuration of all Views
     */

    public void setInitialConf() {
        setOneDofConfig();
        check_1dof.setEnabled(true);
        check_2dof.setEnabled(true);
        check_3dof.setEnabled(true);
        check_1dof.setChecked(true);
        check_2dof.setChecked(false);
        check_3dof.setChecked(false);
        editText_sof1.setText("15");
        config = 1;
    }

    /**
     *This method handles only leave Checkbox 1 (for one
     * degree of freedom) marked, leaving the rest unchecked.
     */
    public void checkOneDof(View view) {
        clearAllValues();
        setOneDofConfig();
        check_1dof.setChecked(true);
        check_2dof.setChecked(false);
        check_3dof.setChecked(false);
    }

    /**
     *This method handles only leave Checkbox 2 (for two
     * degrees of freedom) marked, leaving the rest unchecked.
     */
    public void checkTwoDof(View view) {
        clearAllValues();
        setTwoDofConfig();
        check_1dof.setChecked(false);
        check_2dof.setChecked(true);
        check_3dof.setChecked(false);
    }

    /**
     *This method handles only leave Checkbox 3 (for three
     * degrees of freedom) marked, leaving the rest unchecked.
     */
    public void checkThreeDof(View view) {
        clearAllValues();
        setThreeDofConfig();
        check_1dof.setChecked(false);
        check_2dof.setChecked(false);
        check_3dof.setChecked(true);
    }

    /**
     *This method is responsible for multiply two matrices
     *using the iterative conventional method.
     */
    public static double[][] multiplyMatrix(double[][] a, double[][] b) {
        int rowsInA = a.length;
        int columnsInA = a[0].length;
        int columnsInB = b[0].length;
        double[][] c = new double[rowsInA][columnsInB];
        for (int i = 0; i < rowsInA; i++) {
            for (int j = 0; j < columnsInB; j++) {
                for (int k = 0; k < columnsInA; k++) {
                    c[i][j] = c[i][j] + a[i][k] * b[k][j];
                }
            }
        }
        return c;
    }
}
