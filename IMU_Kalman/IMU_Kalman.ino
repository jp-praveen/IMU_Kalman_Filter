// Author: Praveen Jawaharlal Ayyanathan
// This project implements a Kalman Filter for sensor fusion on an MPU9250 IMU to estimate roll, pitch, and yaw angles. 
// It combines data from the accelerometer, gyroscope, and magnetometer to provide orientation estimates in real-time.
// Kalman Filter: Implements a 6-state (roll, roll rate, pitch, pitch rate, yaw, yaw rate) Kalman filter for orientation estimation.
// In the code, the state vector is represented by X_BEST_ESTIMATE.
// Bias Calibration: Calibrates the accelerometer and gyroscope biases during setup.


#include <math.h>
#include "MPU9250.h"

// NECESSARY CONSTANTS FOR IMU
int FLAG = 1;
float P[6][6] = {{1/1000000,0,0,0,0,0},{0,1/1000000,0,0,0,0},{0,0,1/1000000,0,0,0},{0,0,0,1/1000000,0,0},{0,0,0,0,1/1000000,0},{0,0,0,0,0,1/1000000}};
float X_BEST_ESTIMATE[6][1];
float PHI_FUNC[1][1],THETA_FUNC[1][1], PSI_FUNC[1][1];
float ACC_BIAS[3][1] = {{0},{0},{0}};
float GYRO_BIAS[3][1] = {{0},{0},{0}};

// NECESSARY VARIABLES FOR IMU
unsigned long OLD_TIME_IMU; 

// MPU9250 OBJECT WITH THE SENSOR ON I2C BUS 0 WITH ADDRESS 0x68
MPU9250 IMU(Wire,0x68);

void setup() { 
  Serial.begin(9600);
  IMU.begin();
  ACC_GYRO_CAL( ACC_BIAS, GYRO_BIAS );
  OLD_TIME_IMU = millis();
  
}

void loop() {

  // Sensor Calibration and Kalman Filter functions
  Kalman0( PHI_FUNC,THETA_FUNC, PSI_FUNC, ACC_BIAS );
  Kalman1( PHI_FUNC,THETA_FUNC, PSI_FUNC, P, OLD_TIME_IMU, X_BEST_ESTIMATE, GYRO_BIAS, FLAG);
  Kalman2(P, X_BEST_ESTIMATE, S_T, Y_T);

  Serial.print(X_BEST_ESTIMATE[0][0]*180/3.415);
  Serial.print("\t");
  Serial.print(X_BEST_ESTIMATE[2][0]*180/3.415);
  Serial.print("\t");
  Serial.print(X_BEST_ESTIMATE[4][0]*180/3.415);
  Serial.print("\n");
  delay(100);
      
}

// Accelerometer and Gyroscope Calibration
float ACC_GYRO_CAL( float ACC_BIAS[3][1], float GYRO_BIAS[3][1] ){
  Serial.print("Starting Accelerometer and Gyroscope Calibration.....");
  Serial.print("\n");

  for(int j=0;j<1000;j++){
    // Read accelerometer and Gyroscrope data
    IMU.readSensor();
    ACC_BIAS[0][0] = ACC_BIAS[0][0] -IMU.getAccelX_mss();
    ACC_BIAS[1][0] = ACC_BIAS[1][0] -IMU.getAccelY_mss();
    ACC_BIAS[2][0] = ACC_BIAS[2][0] -IMU.getAccelZ_mss();
    GYRO_BIAS[0][0] = GYRO_BIAS[0][0] -IMU.getGyroX_rads();
    GYRO_BIAS[1][0] = GYRO_BIAS[1][0] -IMU.getGyroY_rads();
    GYRO_BIAS[2][0] = GYRO_BIAS[2][0] -IMU.getGyroZ_rads();
  }
  ACC_BIAS[0][0] = ACC_BIAS[0][0]/1000;
  ACC_BIAS[1][0] = ACC_BIAS[1][0]/1000;
  ACC_BIAS[2][0] = ACC_BIAS[2][0]/1000 - 9.81;
  GYRO_BIAS[0][0] = GYRO_BIAS[0][0]/1000;
  GYRO_BIAS[1][0] = GYRO_BIAS[1][0]/1000;
  GYRO_BIAS[2][0] = GYRO_BIAS[2][0]/1000;
  Serial.print("Accelerometer and Gyroscope Calibration completed.....");
  Serial.print("\n");
}

float Kalman0( float PHI_FUNC[1][1], float THETA_FUNC[1][1], float PSI_FUNC[1][1], float ACC_BIAS[3][1] ){
  
  float ACC[3][1], GYRO[3][1], MAGNETOMETER[1][3], MAGX,MAGY,MAGZ,MAG[1][3];
  
  // Data Collection
  IMU.readSensor();
  ACC[0][0] = -IMU.getAccelX_mss() - ACC_BIAS[0][0] ;
  ACC[1][0] = -IMU.getAccelY_mss() - ACC_BIAS[1][0];
  ACC[2][0] = -IMU.getAccelZ_mss() - ACC_BIAS[2][0];
  
  MAGNETOMETER[0][0] = IMU.getMagX_uT();//-1.75; // -2;
  MAGNETOMETER[0][1] = IMU.getMagY_uT();//-23.5; //-20;
  MAGNETOMETER[0][2] = IMU.getMagZ_uT();//+21.67; //+20;

  // Magnetometer data calibration
  MAGNETOMETER[0][0] = MAGNETOMETER[0][0] - 7.502051994 ;//- 14.74572987;
  MAGNETOMETER[0][1] = MAGNETOMETER[0][1] - 60.93806745 ;//- 53.84865720;
  MAGNETOMETER[0][2] = MAGNETOMETER[0][2] - (-57.36255207);//- (-56.76457803);

  multiply_magcal(MAGNETOMETER, MAG);

  MAGX = MAG[0][0];
  MAGY = MAG[0][1];
  MAGZ = MAG[0][2];
  MAG[0][0] = MAGY/sqrt(MAGX*MAGX + MAGY*MAGY + MAGZ*MAGZ);
  MAG[0][1] = MAGX/sqrt(MAGX*MAGX + MAGY*MAGY + MAGZ*MAGZ);
  MAG[0][2] = -MAGZ/sqrt(MAGX*MAGX + MAGY*MAGY + MAGZ*MAGZ);
  
  PHI_FUNC[0][0] = atan2(ACC[1][0], sqrt(ACC[0][0]*ACC[0][0] + ACC[2][0]*ACC[2][0]));
  THETA_FUNC[0][0] = atan2(-ACC[0][0], sqrt(ACC[1][0]*ACC[1][0] + ACC[2][0]*ACC[2][0]));
  
  PSI_FUNC[0][0] = atan2(-MAG[0][1]*cos(PHI_FUNC[0][0]) + MAG[0][2]*sin(PHI_FUNC[0][0]), MAG[0][0]*cos(THETA_FUNC[0][0]) + MAG[0][1]*sin(THETA_FUNC[0][0])*sin(PHI_FUNC[0][0]) + MAG[0][2]*sin(THETA_FUNC[0][0])*cos(PHI_FUNC[0][0]));
}

// Kalman Filter
float Kalman1(float PHI_FUNC[1][1], float THETA_FUNC[1][1], float PSI_FUNC[1][1],  float P[6][6], unsigned long  OLD_TIME_IMU, float X_BEST_ESTIMATE[6][1], float GYRO_BIAS[3][1], int FLAG ){

  float EULER_DOT[3][1],P1[6][6], C1[3][6], C2[3][3], GYRO[3][1],X_PREV_BEST_ESTIMATE[6][1];
  float S_T[3][3], Y_T[3][1];
  int C[3][6] ={{1, 0, 0, 0, 0, 0},{0, 0, 1, 0, 0, 0},{0, 0, 0, 0, 1, 0}};
  int C_TRANS[6][3] = {{1, 0, 0},{0, 0, 0},{0, 1, 0},{0, 0, 0},{0, 0, 1},{0, 0, 0}};
  //float BIAS_GYROX = 0.001902167803223, BIAS_GYROY = -0.007651422990139, BIAS_GYROZ = 0.000201257808508; 

  IMU.readSensor();
  GYRO[0][0] = -IMU.getGyroX_rads() - GYRO_BIAS[0][0];
  GYRO[1][0] = -IMU.getGyroY_rads() - GYRO_BIAS[1][0];
  GYRO[2][0] = -IMU.getGyroZ_rads() - GYRO_BIAS[2][0];
  
  unsigned long NEW_TIME_IMU, dt;

  NEW_TIME_IMU = millis();
  dt = NEW_TIME_IMU - OLD_TIME_IMU; 
  OLD_TIME_IMU = NEW_TIME_IMU;
  float dt_float = (float) dt;
  dt_float = dt_float/1000;

  float A[6][6] = {{1, -dt_float, 0,  0, 0, 0},{0,  1,  0,  0, 0, 0},{0,  0,  1, -dt_float, 0, 0},{ 0,  0,  0,  1, 0, 0},{0,  0,  0,  0, 1, -dt_float},{0,  0,  0,  0, 0, 1}};
  float A_TRANS[6][6] = {{1, 0,   0, 0,   0, 0},{-dt_float, 1,   0, 0,   0, 0},{0, 0,   1, 0,   0, 0},{0, 0, -dt_float, 1,   0, 0},{0, 0,   0, 0,   1, 0},{0, 0,   0, 0, -dt_float, 1}};
  float B[6][3] = {{dt_float, 0, 0},{0,  0, 0},{0,  dt_float, 0},{0,  0, 0},{0, 0, dt_float},{0,  0, 0}};
  
  
  float DOT_MATRIX[3][3] = {{1, sin(PHI_FUNC[0][0])*tan(THETA_FUNC[0][0]), cos(PHI_FUNC[0][0])*tan(THETA_FUNC[0][0])},{0, cos(PHI_FUNC[0][0]),-sin(PHI_FUNC[0][0])},{0, sin(PHI_FUNC[0][0])/cos(THETA_FUNC[0][0]), cos(PHI_FUNC[0][0])/cos(THETA_FUNC[0][0])}};
  multiply_eulerdot(DOT_MATRIX, GYRO, EULER_DOT);    

  if(FLAG == 1){ 
    X_PREV_BEST_ESTIMATE[0][0] = 0.;
    X_PREV_BEST_ESTIMATE[1][0] = GYRO_BIAS[0][0];
    X_PREV_BEST_ESTIMATE[2][0] = 0.;
    X_PREV_BEST_ESTIMATE[3][0] = GYRO_BIAS[1][0];
    X_PREV_BEST_ESTIMATE[4][0] = 0.;
    X_PREV_BEST_ESTIMATE[5][0] = GYRO_BIAS[2][0];

    FLAG = 2;
  } else if(FLAG==2){
    X_PREV_BEST_ESTIMATE[0][0] = X_BEST_ESTIMATE[0][0];
    X_PREV_BEST_ESTIMATE[1][0] = X_BEST_ESTIMATE[1][0];
    X_PREV_BEST_ESTIMATE[2][0] = X_BEST_ESTIMATE[2][0];
    X_PREV_BEST_ESTIMATE[3][0] = X_BEST_ESTIMATE[3][0];
    X_PREV_BEST_ESTIMATE[4][0] = X_BEST_ESTIMATE[4][0];
    X_PREV_BEST_ESTIMATE[5][0] = X_BEST_ESTIMATE[5][0];
  }
  
  for(int i = 0; i < 6; i++) {
    X_BEST_ESTIMATE[i][0] = A[i][0]*X_PREV_BEST_ESTIMATE[0][0] + A[i][1]*X_PREV_BEST_ESTIMATE[1][0] + A[i][2]*X_PREV_BEST_ESTIMATE[2][0] + A[i][3]*X_PREV_BEST_ESTIMATE[3][0] + A[i][4]*X_PREV_BEST_ESTIMATE[4][0] + A[i][5]*X_PREV_BEST_ESTIMATE[5][0] + B[i][0]*EULER_DOT[0][0] + B[i][1]*EULER_DOT[1][0] + B[i][2]*EULER_DOT[2][0]; 
    //X_BEST_ESTIMATE[i][0] = -0.5;
  }
    
  multiply_AwithP(A, P, P1);    
  multiply_P1withAtrans(P1, A_TRANS, P);    

  Y_T[0][0] = PHI_FUNC[0][0] - (C[0][0]*X_BEST_ESTIMATE[0][0] + C[0][1]*X_BEST_ESTIMATE[1][0] + C[0][2]*X_BEST_ESTIMATE[2][0] + C[0][3]*X_BEST_ESTIMATE[3][0] + C[0][4]*X_BEST_ESTIMATE[4][0] + C[0][5]*X_BEST_ESTIMATE[5][0]);
  Y_T[1][0] = THETA_FUNC[0][0] - (C[1][0]*X_BEST_ESTIMATE[0][0] + C[1][1]*X_BEST_ESTIMATE[1][0] + C[1][2]*X_BEST_ESTIMATE[2][0] + C[1][3]*X_BEST_ESTIMATE[3][0] + C[1][4]*X_BEST_ESTIMATE[4][0] + C[1][5]*X_BEST_ESTIMATE[5][0]);
  Y_T[2][0] = PSI_FUNC[0][0] - (C[2][0]*X_BEST_ESTIMATE[0][0] + C[2][1]*X_BEST_ESTIMATE[1][0] + C[2][2]*X_BEST_ESTIMATE[2][0] + C[2][3]*X_BEST_ESTIMATE[3][0] + C[2][4]*X_BEST_ESTIMATE[4][0] + C[2][5]*X_BEST_ESTIMATE[5][0]);

  multiply_CwithP(C, P, C1);
  multiply_CwithCTrans(C1, C_TRANS, S_T);

  Kalman2(P, X_BEST_ESTIMATE, S_T, Y_T);
  GYRO_BIAS[0][0] = X_BEST_ESTIMATE[1][0];
  GYRO_BIAS[1][0] = X_BEST_ESTIMATE[3][0];
  GYRO_BIAS[2][0] = X_BEST_ESTIMATE[5][0];
}
  
// Kalman Filter
float Kalman2( float P[6][6], float X_BEST_ESTIMATE[6][1], float S_T[3][3], float Y_T[3][1] ){
  float S_T_INV[3][3], S_T_DET[1][1], S_T_ADJ[3][3];
  float K_GAIN[6][3], KT_YT[6][1], K1[6][3], P3[6][6], P4[6][6] ;
  int I[6][6] = {{1,0,0,0,0,0},{0,1,0,0,0,0},{0,0,1,0,0,0},{0,0,0,1,0,0},{0,0,0,0,1,0},{0,0,0,0,0,1}};
  float K_C[6][6];
  int C[3][6] ={{1, 0, 0, 0, 0, 0},{0, 0, 1, 0, 0, 0},{0, 0, 0, 0, 1, 0}};
  int C_TRANS[6][3] = {{1, 0, 0},{0, 0, 0},{0, 1, 0},{0, 0, 0},{0, 0, 1},{0, 0, 0}};
  
  adjoint_3by3(S_T, S_T_ADJ);
  determinant_3by3(S_T, S_T_DET);
  
  inverse_3by3(S_T_ADJ, S_T_DET,S_T_INV);
  
  multiply_PwithCTrans(P, C_TRANS, K1);
  multiply_K1withSTinv(K1, S_T_INV, K_GAIN);

\\ Mathematical Functions - Matrix multiplication, adjoint calculation
  multiply_KTwithYT(K_GAIN, Y_T, KT_YT);
  for(int i = 0; i < 6; i++) {
    X_BEST_ESTIMATE[i][0] += KT_YT[i][0];
    //X_PREV_BEST_ESTIMATE[i][0] =  X_BEST_ESTIMATE[i][0];     
  }
  
  multiply_KwithC(K_GAIN, C, K_C);
     
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 6; j++){
      P4[i][j] = I[i][j] - K_C[i][j]; 
    }
  }

  multiply_PwithP(P4, P, P3);
  
  for(int i = 0; i < 6; i++) {
    for(int j = 0; j < 6; j++){
      P[i][j] = P3[i][j]; 
    }
  }
  return;
}

float multiply_magcal(float mat1[1][3], float MAG[1][3]){
  int m1 = 1,m2 = 3;
 //float A_MAG_CAL[3][3] = {{0.939579873911505,0.0157380187228683,0.0267911217602576},{0.0157380187228683,1.00533490810877,-0.0643697912001522},{0.0267911217602576,-0.0643697912001522,1.06387946185326}};
 float A_MAG_CAL[3][3] = {{0.914721236717404,0.006591763526623,0.011361282977070},{0.006591763526623,1.070038649189700,-0.035519277139419},{0.011361282977070,-0.035519277139419,1.023043668814983}};

 int n1 = 3, n2 = 3;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        MAG[j][k] = 0;
        for(int i=0;i<n1;i++){
          MAG[j][k] += mat1[j][i]*A_MAG_CAL[i][k];             
        }
      }
    }
  }
  return;  
}

float multiply_eulerdot(float mat1[3][3], float mat2[3][1], float EULER_DOT[3][1]){
  int m1 = 3,m2 = 3,n1 = 3,n2 = 1;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        EULER_DOT[j][k] = 0;
        for(int i=0;i<n1;i++){
          EULER_DOT[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }  
  return;
}

float multiply_AwithP(float mat1[6][6], float mat2[6][6], float P1[6][6]){
  int m1=6,m2=6,n1=6,n2=6;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        P1[j][k] = 0;
        for(int i=0;i<n1;i++){
          P1[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }
  return;  
}

float multiply_P1withAtrans(float mat1[6][6], float mat2[6][6], float P[6][6]){
  int m1=6,m2=6,n1=6,n2=6;
  //int Q[6][6] = {{1000000,0,0,0,0,0},{0,1000000,0,0,0,0},{0,0,1000000,0,0,0},{0,0,0,1000000,0,0},{0,0,0,0,1000000,0},{0,0,0,0,0,1000000}};
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        if(j==k){
          P[j][k] = 1000000;
        }
        else {
          P[j][k] = 0;
        }
        for(int i=0;i<n1;i++){
          P[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }
  return;  
}

float multiply_CwithP(int mat1[3][6], float mat2[6][6], float C1[3][6]){
  /*Serial.print(m1);
  Serial.print("\t");
  Serial.print(n2);
  Serial.print("\n");*/
  int m1 = 3,m2 = 6,n1 = 6,n2 = 6;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        C1[j][k] = 0;
        for(int i=0;i<n1;i++){
          C1[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }
 
     
}

float multiply_CwithCTrans(float mat1[3][6], int mat2[6][3], float S_T[3][3]){
  int m1=3,n2 = 3,m2=6,n1=6;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        if(j==k){
          S_T[j][k] = 1;          
        }
        else {
          S_T[j][k] = 0;
        }
        for(int i=0;i<n1;i++){
          S_T[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }
  
}

float adjoint_3by3(float mat[3][3], float S_T_ADJ[3][3]){
  float TEMP[3][3];
  TEMP[0][0] = mat[1][1]*mat[2][2] - mat[1][2]*mat[2][1];
  TEMP[0][1] = - (mat[1][0]*mat[2][2] - mat[2][0]*mat[1][2]);
  TEMP[0][2] = mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0];
  TEMP[1][0] = - (mat[0][1]*mat[2][2] - mat[0][2]*mat[2][1]);
  TEMP[1][1] = mat[0][0]*mat[2][2] - mat[2][0]*mat[0][2];
  TEMP[1][2] = - (mat[0][0]*mat[2][1] - mat[0][1]*mat[2][0]);
  TEMP[2][0] = mat[0][1]*mat[1][2] - mat[0][2]*mat[1][1];
  TEMP[2][1] = - (mat[0][0]*mat[1][2] - mat[0][2]*mat[1][0]);
  TEMP[2][2] = mat[0][0]*mat[1][1] - mat[0][1]*mat[1][0];

  for(int j=0;j<3;j++){
    for(int k=0;k<3;k++){
      S_T_ADJ[j][k] = TEMP[k][j];
    }
  }    
}

float determinant_3by3(float mat[3][3], float S_T_DET[1][1]){
  S_T_DET[0][0] = mat[0][0]*(mat[1][1]*mat[2][2] - mat[1][2]*mat[2][1]) - mat[0][1]*(mat[1][0]*mat[2][2] - mat[1][2]*mat[2][0]) + mat[0][2]*(mat[1][0]*mat[2][1] - mat[1][1]*mat[2][0]);
  //Serial.print(DETERMINANT,3);
  //Serial.print("\n");
}

float inverse_3by3(float S_T_ADJ[3][3], float S_T_DET[1][1], float S_T_INV[3][3]){
  for(int j=0;j<3;j++){  
    for(int k=0;k<3;k++){
      S_T_INV[j][k] = S_T_ADJ[j][k]/S_T_DET[0][0];
    }
  }
  
}

float multiply_PwithCTrans(float mat1[6][6], int mat2[6][3], float K1[6][3]){
  int m1 = 6,m2 = 6,n1 = 6,n2 = 3;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        K1[j][k] = 0;
        for(int i=0;i<n1;i++){
          K1[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }  
}

float multiply_K1withSTinv(float mat1[6][3], float mat2[3][3], float K_GAIN[6][3]){
  int m1 = 6,m2 = 3,n1 = 3,n2 = 3;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        K_GAIN[j][k] = 0;
        for(int i=0;i<n1;i++){
          K_GAIN[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }  
}

float multiply_KTwithYT(float mat1[6][3], float mat2[3][1], float KT_YT[6][1]){
  int m1 = 6,m2 = 3,n1 = 3,n2 = 1;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        KT_YT[j][k] = 0;
        for(int i=0;i<n1;i++){
          KT_YT[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }  
}

float multiply_KwithC(float mat1[6][3], int mat2[3][6], float K_C[6][6]){
  int m1 = 6,m2 = 3,n1 = 3,n2 = 6;
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        K_C[j][k] = 0;
        for(int i=0;i<n1;i++){
          K_C[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }
  
}

float multiply_PwithP(float mat1[6][6], float mat2[6][6], float P3[6][6]){
  int m1 = 6, m2 = 6, n1 = 6, n2 = 6;    
  if (m2==n1){
    for(int j=0;j<m1;j++){      
      for(int k=0;k<n2;k++){
        P3[j][k] = 0;
        for(int i=0;i<n1;i++){
          P3[j][k] += mat1[j][i]*mat2[i][k];             
        }
      }
    }
  }
  
}
