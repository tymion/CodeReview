#ifndef HEADER_STRUCTURS_H_
#define HEADER_STRUCTURS_H_

#define SIZE 9

typedef struct {
  double arr[SIZE];
  int row;
  int col;
} MatrixMath_s;

typedef struct {
  double arr[16];
  int row;
  int col;
} MatrixMath4x4_s;

typedef struct {
  double arr[36];
  int row;
  int col;
} MatrixMath6x6_s;

typedef struct {
  double arr[54];
  int row;
  int col;
} MatrixMath9x6_s;

typedef struct {
  double arr[81];
  int row;
  int col;
} MatrixMath9x9_s;

typedef struct {
  double arr[4];
} Quaternion_s;

typedef struct {
  double arr[7];
} MatrixMath7x1_s;

typedef struct {
  double arr[9];
} MatrixMath9x1_s;

typedef struct {
  MatrixMath_s b_eci;
  MatrixMath_s b_eciLower;
} igrf_output;

typedef struct {
  int year;
  int month;
  int day;
  int hour;
  int min;
  int sec;
} datetime;

typedef struct {
  double k;
  double kp;
  double kn;
} ctrlgains_s;

#endif  // HEADER_STRUCTURS_H_
