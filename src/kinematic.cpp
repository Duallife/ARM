#include "kinematic.h"

MatrixUtils mat;
double T[4][4];
double R[3][3];
double P[3][1];
double Euler[3];

const double AB = 91.000;
const double BC = 123.809;
const double AC = 180.354;
double WRIST = 71.0;

void printMatrix(double* T, int r, int c){
    mat.print_matrix(T, r, c);
}

void rotm2eul(double R[3][3], double euler[3]) {
    euler[0] = atan2(R[2][1], R[2][2]); // Roll
    euler[1] = atan2(-R[2][0], sqrt(R[2][1]*R[2][1] + R[2][2]*R[2][2])); // Pitch
    euler[2] = atan2(R[1][0], R[0][0]); // Yaw
}

void eul2rotm(double euler[3], double R[3][3]) {
    // Extract the Euler angles
    double roll = euler[0];  // x-axis rotation
    double pitch = euler[1]; // y-axis rotation
    double yaw = euler[2];   // z-axis rotation

    // Precompute cosines and sines of the Euler angles
    double c1 = cos(yaw);  // cos(psi)
    double s1 = sin(yaw);  // sin(psi)
    double c2 = cos(pitch); // cos(theta)
    double s2 = sin(pitch); // sin(theta)
    double c3 = cos(roll);  // cos(phi)
    double s3 = sin(roll);  // sin(phi)

    // Compute the rotation matrix elements
    R[0][0] = c1 * c2;
    R[0][1] = c1 * s2 * s3 - s1 * c3;
    R[0][2] = c1 * s2 * c3 + s1 * s3;
    R[1][0] = s1 * c2;
    R[1][1] = s1 * s2 * s3 + c1 * c3;
    R[1][2] = s1 * s2 * c3 - c1 * s3;
    R[2][0] = -s2;
    R[2][1] = c2 * s3;
    R[2][2] = c2 * c3;
}

void FK(double inputPose[6]){
    Serial.println("FK started");
    setTheta(inputPose);
    mat.identity(*T, 4);
    mat.print_matrix(*T, 4, 4);
    for (int i = 0; i < 6; i++){
        double alpha = DH_table[i][3];
        double a = DH_table[i][2];
        double d = DH_table[i][1];
        double theta = inputPose[i];

        double A[4][4] = {
            {cos(theta), -sin(theta), 0, a},
            {sin(theta)*cos(alpha), cos(theta)*cos(alpha), -sin(alpha), -sin(alpha)*d},
            {sin(theta)*sin(alpha), cos(theta)*sin(alpha), cos(alpha), cos(alpha)*d},
            {0, 0, 0, 1}
        };
        
        // mat.mul_matrix(*T, *A, 4, 4, 4, 4, *T);

        double temp[4][4];

        for (int m = 0; m < 4; m++) {
            for (int n = 0; n < 4; n++) {
                temp[m][n] = 0;
                for (int k = 0; k < 4; k++) {
                    temp[m][n] += T[m][k] * A[k][n];
                }
            }
        }

        for (int m = 0; m < 4; m++) {
            for (int n = 0; n < 4; n++) {
                T[m][n] = temp[m][n];
            }
        }
    }
    mat.get_rot_mat(*T, *R);
    mat.get_pos_vec(*T, *P);
    rotm2eul(R, Euler);
    // Serial.println("Transform Matrix : ");
    // mat.print_matrix(*T, 4, 4);
    Serial.println("Rotation Matrix : ");
    mat.print_matrix(*R, 3, 3);
    Serial.println("Position Vector : ");
    mat.print_matrix(*P, 3, 1);
    Serial.println("Euler Angles : ");
    mat.print_matrix(Euler, 3, 1);
    mat.copy_matrix(*T, 4, 4, *target);

};

void copySubArray(double source[4][4], double dest[3][3]) {
    for(int i = 0; i < 3; i++) {
        for(int j = 0; j < 3; j++) {
            dest[i][j] = source[i][j];
        }
    }
}

void IK(double target[4][4]){
    Serial.println("IK start");
    double p[3] = {target[0][3], target[1][3], target[2][3]};
    double pw[3] = {p[0] - WRIST*target[0][2], p[1] - WRIST*target[1][2], p[2] - WRIST*target[2][2]};

    double AC2 = sqrt(pw[0]*pw[0] + pw[1]*pw[1] + pw[2]*pw[2]);
    double BC2 = sqrt(sq(pw[0]) + sq(pw[1]) + sq(pw[2]-AB));
    double ctheta3 = acos((AB*AB + BC*BC - AC2*AC2)/(2*AB*BC));
    double cthetaB = 1.97745;
    double cthetaA = atan(pw[2]/hypot(pw[0], pw[1]));
    double theta1 = atan(pw[1]/pw[0]);
    double theta2 = acos((AB*AB + AC2*AC2 - BC*BC)/(2*AB*AC2)) + cthetaA;
    double theta3 = ctheta3 - cthetaB;
    double R06[3][3];
    copySubArray(target, R06);

    double R03[3][3] = {{cos(theta2 + theta3)*cos(theta1), sin(theta1),  sin(theta2 + theta3)*cos(theta1)},
                       {cos(theta2 + theta3)*sin(theta1), -cos(theta1),  sin(theta2 + theta3)*sin(theta1)},
                       {sin(theta2 + theta3),              0          ,            -cos(theta2 + theta3)}};
    double R03_inv[3][3];
    double R36[3][3];
    mat.transpose(*R03, 3, 3, *R03_inv);
    mat.mul_matrix(*R03_inv, *R06, 3, 3, 3, 3, *R36);
    double theta4 = atan2(R36[1][2], R36[0][2]);
    double theta5 = atan2(hypot(R36[1][2],R36[0][2]), R36[2][2]);
    double theta6 = atan2(R36[2][1], -R36[2][0]);

    double thetaAll[6] = {theta1, theta2, theta3, theta4, theta5, theta6};
    Serial.println("Inverse Kinematics angle: ");
    mat.print_matrix(thetaAll, 1, 6);
    setTheta(thetaAll);

};
