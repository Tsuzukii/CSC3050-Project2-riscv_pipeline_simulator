#include <math.h>
#include "lib.h"

#define INPUT_NEURONS 2
#define HIDDEN_NEURONS 3
#define OUTPUT_NEURONS 1

int weights_input_hidden[INPUT_NEURONS][HIDDEN_NEURONS] = {
    {1, 2, -3},
    {-7, -3, 3}
};
int weights_hidden_output[HIDDEN_NEURONS][OUTPUT_NEURONS] = {
    {4},
    {-6},
    {6}
};

int input[INPUT_NEURONS];
int hidden[HIDDEN_NEURONS];
int output[OUTPUT_NEURONS];

// Step activation function
int step(int x) {
    return x >= 0 ? 1 : 0;
}

int elu(int x) {
    return x > 0 ? x : (exp(x) - 1);
}

// Forward pass from input to hidden layer
void forwardPassInputToHidden() {
    for (int i = 0; i < HIDDEN_NEURONS; i++) {
        hidden[i] = 0;
        for (int j = 0; j < INPUT_NEURONS; j++) {
            // Multiply inputs by their corresponding weights
            // Since we're using integers, we divide by 10 to simulate smaller weight changes
            hidden[i] += (input[j] * weights_input_hidden[j][i]);
        }
        // Apply the step activation function
        hidden[i] = elu(hidden[i]);
    }
}

// Forward pass from hidden to output layer
void forwardPassHiddenToOutput() {
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        output[i] = 0;
        for (int j = 0; j < HIDDEN_NEURONS; j++) {
            // Multiply hidden layer outputs by their corresponding weights
            // and divide by 10 again to simulate smaller weight changes
            output[i] += (hidden[j] * weights_hidden_output[j][i]);
        }
        // Apply the step activation function
        output[i] = elu(output[i]);
    }
}

int main() {
    // Example integer input values (could be 0 or 1)
    input[0] = 4;
    input[1] = 3;

    // Perform forward pass
    forwardPassInputToHidden();
    forwardPassHiddenToOutput();

    // Print the result
    print_s("Output from simplified neural network: \n");
    for (int i = 0; i < OUTPUT_NEURONS; i++) {
        print_d(output[i]);
        print_s("\n");
    }

    return 0;
}