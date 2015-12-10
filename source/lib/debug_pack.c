void displayQuaternion(Quaternion_s* A) {
  printf("\n\n displayQuaternion:\n");
  int i;
  for (i = 0; i < 4; i++)
    printf("%10.10f\t ", (*A).arr[i]);
  printf("\n ---------------------- \n");
}

void displayMatrix(MatrixMath_s* A) {
  printf("\n\n displayMatrix:\n");
  int i, k;
  for (i = 0; i < (*A).row; i++) {
    for (k = 0; k < (*A).col; k++)
      printf("%10.10f\t ", (*A).arr[i*3+k]);
    printf("\n");
  }
  printf("\n ---------------------- \n");
}

void displayArray(MatrixMath_s* A) {
  printf("\n\n displayArray:\n");
  int w = 0;
  for (w = 0; w < 9; w++)
    printf("%10.10f\t ", (*A).arr[w]);
  printf("\n ---------------------- \n");
}

void displayMatrix7x1(MatrixMath7x1_s* A) {
  printf("\n\n displayMatrix7x1:\n");
  int i;
  for (i = 0; i < 7; i++)
    printf("%10.10f\t ", (*A).arr[i]);
  printf("\n ---------------------- \n");
}

void displayMatrix9x1(MatrixMath9x1_s* A) {
  printf("\n\n displayMatrix9x1:\n");
  int i;
  for (i = 0; i < 9; i++)
    printf("%10.10f\t ", (*A).arr[i]);
  printf("\n ---------------------- \n");
}

void displayMatrix6x6(MatrixMath6x6_s* A) {
  printf("\n\n displayMatrix6x6:\n");
  int i, k;
  for (i = 0; i < 6; i++) {
    for (k = 0; k < 6; k++)
      printf("%10.10f\t ", (*A).arr[i*6+k]);
    printf("\n");
  }
  printf("\n ---------------------- \n");
}

void displayMatrix9x9(MatrixMath9x9_s* A) {
  printf("\n\n displayMatrix9x9:\n");
  int i, k;
  for (i = 0; i < 9; i++) {
    for (k = 0; k < 9; k++)
      printf("%10.10f\t ", (*A).arr[i*9+k]);
    printf("\n");
  }
  printf("\n ---------------------- \n");
}

void displayMatrix4x4(MatrixMath4x4_s* A) {
  printf("\n\n displayMatrix4x4:\n");
  int i, k;
  for (i = 0; i < (*A).row; i++) {
    for (k = 0; k < (*A).col; k++)
      printf("%10.10f\t ", (*A).arr[i*4+k]);
    printf("\n ---------------------- \n");
  }
}
