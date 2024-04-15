// C Program to find transpose 
// of a square matrix 
#include "lib.h"
#define N 4

// This function stores transpose  of A[][] in B[][] 
void transpose(int A[][N], int B[][N]) 
{ 
	int i, j; 
	for (i = 0; i < N; i++) 
		for (j = 0; j < N; j++) 
			// Assigns the transpose of element A[j][i] to B[i][j] 
			B[i][j] = A[j][i]; 
} 

// Driver code 
int main() 
{ 
	int A[N][N] = { { 1, 1, 1, 1 }, 
					{ 2, 2, 2, 2 }, 
					{ 3, 3, 3, 3 }, 
					{ 4, 4, 4, 4 } }; 

	int B[N][N], i, j; 

	print_s("Original matrix is \n"); 
	for (i = 0; i < N; i++) { 
		for (j = 0; j < N; j++) {
			print_d(A[i][j]);
			print_c(' ');
		}
		print_c('\n'); 
	}

	transpose(A, B); 

	print_s("Result matrix is \n"); 
	for (i = 0; i < N; i++) { 
		for (j = 0; j < N; j++){
			print_d(B[i][j]);
			print_c(' ');
		}
		print_c('\n'); 
	}

	return 0; 
}
