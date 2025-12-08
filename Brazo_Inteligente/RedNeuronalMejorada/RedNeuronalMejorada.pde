int cols = 28, rows = 28; // 28x28 grid
int pixelSize = 20;       // Size of each square
int brushSize = 1;        // Brush size (3x3 grid for example)
float[][] grid = new float[cols][rows]; // Stores the drawn values

//Weights and biases
float[][] w1, w2, w3;
float[] b1, b2, b3;


float[] hidden_layer_1;
float[] hidden_layer_2;
float[] output_layer;
int prob0, prob1, prob2, prob3, prob4, prob5, prob6, prob7, prob8, prob9;

void settings() {
  size(cols * pixelSize, rows * pixelSize);
}


import processing.serial.*;

Serial myPort;

void setup() {
  size(cols * pixelSize, rows * pixelSize);
  background(255); // White background
  String portName = Serial.list()[0]; // Selecciona el primer puerto (ajusta si es otro)
  myPort = new Serial(this, portName, 9600);
  
  // Load weights and biases
  w1 = loadWeights("weights_layer_1.txt", 784, 128);  // 128 neurons, 784 input features
  b1 = loadBiases("biases_layer_1.txt", 128);          // 128 biases for first layer
  w2 = loadWeights("weights_layer_2.txt", 128, 64);   // 128 neurons, 64 output neurons
  b2 = loadBiases("biases_layer_2.txt", 64);          // 64 biases for hidden layer
  w3 = loadWeights("weights_layer_3.txt", 64, 10);   // 64 neurons, 10 output neurons
  b3  = loadBiases("biases_layer_3.txt", 10);          // 10 biases for output layer
  
}

void draw() {
  drawGrid();
}

// Function to load weights (ensure the file has enough lines)
  float[][] loadWeights(String filename, int rows, int cols) {
    String[] lines = loadStrings(filename);
    if (lines.length != rows) {
      println("Error: Expected " + rows + " rows, but found " + lines.length);
      exit();  // Exit if file size mismatch
    }
    
    float[][] weights = new float[rows][cols];
    
    for (int i = 0; i < rows; i++) {
      String[] values = split(lines[i], ",");
      if (values.length != cols) {
        println("Error: Expected " + cols + " columns, but found " + values.length + " in row " + i);
        exit();  // Exit if file size mismatch
      }
      for (int j = 0; j < cols; j++) {
        weights[i][j] = float(values[j]);
      }  
    }
    return weights;
  }
  
  // Function to load biases (ensure the file has enough lines)
  float[] loadBiases(String filename, int size) {
    String[] lines = loadStrings(filename);
    if (lines.length != size) {
      println("Error: Expected " + size + " values, but found " + lines.length);
      exit();  // Exit if file size mismatch
    }
    
    float[] biases = new float[size];
    for (int i = 0; i < size; i++) {
      biases[i] = float(lines[i]);
    }
    return biases;
  }
  
  // ReLU activation function
  float[] relu(float[] inputs) {
    float[] outputs = new float[inputs.length];
    for (int i = 0; i < inputs.length; i++) {
      outputs[i] = max(0, inputs[i]); // ReLU activation
    }
    return outputs;
  }
  
  // Softmax activation function
  float[] softmax(float[] inputs) {
    float sum = 0;
    float[] expValues = new float[inputs.length];
    
    for (int i = 0; i < inputs.length; i++) {
      expValues[i] = exp(inputs[i]);
      sum += expValues[i];
    }
    
    for (int i = 0; i < inputs.length; i++) {
      expValues[i] /= sum; // Normalize
    }
    return expValues;
  }
  
  float[] normalizeVector(float[] values) {
  int n = values.length;
  float[] normalized = new float[n];
  
  float min = min(values);
  float max = max(values);
  
  for (int i = 0; i < n; i++) {
    normalized[i] = (10 * (values[i] - min) / (max - min));
  }
  
  return normalized;
}

  
  // Matrix multiplication function: performs matrix multiplication and adds biases
  float[] matMul(float[][] weights, float[] inputs, float[] biases) {
    int rows = weights.length;
    println(rows);
    int cols = weights[0].length;
    println(cols);
    float[] outputs = new float[cols];
  
    for (int i = 0; i < cols; i++) {
      float sum = 0;
      for (int j = 0; j < rows; j++) {
        sum += weights[j][i] * inputs[j];
      }
      outputs[i] = sum + biases[i]; // Add bias
    }
    return outputs;
  }


// 🖌️ User draws by clicking or dragging
void mouseDragged() {
  int startX = (mouseX - (brushSize / 2) * pixelSize) / pixelSize;
  int startY = (mouseY - (brushSize / 2) * pixelSize) / pixelSize;
  
   boolean newPixelPainted = false; // Bandera para saber si se pintó un nuevo píxel

  // Loop over the grid cells that fall within the brush area
  for (int dx = 0; dx < brushSize; dx++) {
    for (int dy = 0; dy < brushSize; dy++) {
      int x = startX + dx;
      int y = startY + dy;
      if (x >= 0 && x < cols && y >= 0 && y < rows) {
        if (grid[x][y] == 0) {  // Solo procesamos si el píxel no estaba pintado antes
          grid[x][y] = 1; // Marcamos el píxel como "dibujado"
          newPixelPainted = true; // Se ha pintado un nuevo píxel
          // Example input: a flattened 28x28 image (should be loaded from an actual image)
        float[] input = getFlattenedInput(); // Fill with pixel values (normalized to 0-1)
        
        // Forward pass
        hidden_layer_1 = relu(matMul(w1, input, b1));  // Compute hidden layer
        hidden_layer_2 = relu(matMul(w2, hidden_layer_1, b2));  // Compute hidden layer
        output_layer = normalizeVector(matMul(w3, hidden_layer_2, b3));  // Compute output layer
        }
      }
    }
  }
      
      if (newPixelPainted) {
      // Variables para guardar las probabilidades redondeadas
    prob0 = Math.round(output_layer[0]);
    prob1 = Math.round(output_layer[1]);
    prob2 = Math.round(output_layer[2]);
    prob3 = Math.round(output_layer[3]);
    prob4 = Math.round(output_layer[4]);
    prob5 = Math.round(output_layer[5]);
    prob6 = Math.round(output_layer[6]);
    prob7 = Math.round(output_layer[7]);
    prob8 = Math.round(output_layer[8]);
    prob9 = Math.round(output_layer[9]);

    // Imprimir probabilidades
    println("Probabilidades redondeadas (0-10):");
    println("0: " + prob0, "\n1: " + prob1, "\n2: " + prob2, "\n3: " + prob3, "\n4: " + prob4);
    println("5: " + prob5, "\n6: " + prob6, "\n7: " + prob7, "\n8: " + prob8, "\n9: " + prob9);
    
    // Print predicted probabilities
    float largest = 0;
    int count = 0;
    for (int i = 0; i < output_layer.length; i++) {
    println("Class " + i + ": " + output_layer[i]);
      if(output_layer[i] > largest){
        largest = output_layer[i];
        count = i;
      }
    }
    println("Number : " + count);

 
    int[] secuencia = {
  'n', 'n', 'n', 
  '1', '0' + count,
  '0' + prob0, '0' + prob1, '0' + prob2, '0' + prob3, '0' + prob4,
  '0' + prob5, '0' + prob6, '0' + prob7, '0' + prob8, '0' + prob9
    };

    
    
    for (int i = 0; i < secuencia.length; i++) {
      myPort.write(secuencia[i]);  
      //delay(100); 
    }
  }
        
      
      
}

// 🔳 Draws the 28×28 grid
void drawGrid() {
  for (int x = 0; x < cols; x++) {
    for (int y = 0; y < rows; y++) {
      float value = grid[x][y] * 255; // Convert 0-1 to grayscale
      fill(255 - value); // White = empty, Black = drawn
      stroke(200);
      rect(x * pixelSize, y * pixelSize, pixelSize, pixelSize);
    }
  }
}

// 🔄 Reset grid (Clear drawing)
void keyPressed() {
  if (key == 'c') {
    for (int x = 0; x < cols; x++) {
      for (int y = 0; y < rows; y++) {
        grid[x][y] = 0; // Reset grid
      }
    }
    background(255);
  }
}

// 🖥️ Convert grid to a 1D array for the neural network
float[] getFlattenedInput() {
  float[] input = new float[cols * rows];
  int index = 0;
  for (int y = 0; y < rows; y++) {
    for (int x = 0; x < cols; x++) {
      input[index++] = grid[x][y]; // Store 0 or 1
    }
  }
  return input;
}

// 🖨️ Print the input array (for debugging)
void printInput() {
  float[] input = getFlattenedInput();
  println("Flattened Input:");
  println(join(str(input), ", "));
}

// Press 's' to print the input array
void keyReleased() {
  if (key == 's') {
    
    // Example input: a flattened 28x28 image (should be loaded from an actual image)
    float[] input = getFlattenedInput(); // Fill with pixel values (normalized to 0-1)
    
    // Forward pass
    float[] hidden_layer_1 = relu(matMul(w1, input, b1));  // Compute hidden layer
    float[] hidden_layer_2 = relu(matMul(w2, hidden_layer_1, b2));  // Compute hidden layer
    float[] output_layer = normalizeVector(matMul(w3, hidden_layer_2, b3));  // Compute output layer
  
    // Print predicted probabilities
    float largest = 0;
    int count = 0;
    for (int i = 0; i < output_layer.length; i++) {
    println("Class " + i + ": " + output_layer[i]);
      if(output_layer[i] > largest){
        largest = output_layer[i];
        count = i;
      }
    }
    println("Number : " + count);
  }
}
