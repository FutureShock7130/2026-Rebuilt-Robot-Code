package frc.FSLib.maths;

// This class is generate by Sonnet-4.5 and verified by Wjxcws
public class PolynomialRegression {
  private double[] coefficients;
  private int degree;

  public PolynomialRegression(int degree, double[] x, double[] y) {
    this.degree = degree;
    this.coefficients = new double[degree + 1];
    fit(x, y);
  }

  // Fit the polynomial to the data using least squares method
  private void fit(double[] x, double[] y) {
    if (x.length != y.length) {
      throw new IllegalArgumentException("x and y arrays must have the same length");
    }

    int n = x.length;
    int m = degree + 1;

    // Create the Vandermonde matrix
    double[][] A = new double[n][m];
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < m; j++) {
        A[i][j] = Math.pow(x[i], j);
      }
    }

    // Compute A^T * A
    double[][] ATA = new double[m][m];
    for (int i = 0; i < m; i++) {
      for (int j = 0; j < m; j++) {
        double sum = 0;
        for (int k = 0; k < n; k++) {
          sum += A[k][i] * A[k][j];
        }
        ATA[i][j] = sum;
      }
    }

    // Compute A^T * y
    double[] ATy = new double[m];
    for (int i = 0; i < m; i++) {
      double sum = 0;
      for (int k = 0; k < n; k++) {
        sum += A[k][i] * y[k];
      }
      ATy[i] = sum;
    }

    // Solve the system ATA * coefficients = ATy using Gaussian elimination
    coefficients = solveLinearSystem(ATA, ATy);
  }

  // Predict y value for a given x
  public double predict(double x) {
    double result = 0;
    for (int i = 0; i <= degree; i++) {
      result += coefficients[i] * Math.pow(x, i);
    }
    return result;
  }

  // Get the coefficients
  public double[] getCoefficients() {
    return coefficients.clone();
  }

  // Solve linear system using Gaussian elimination
  private double[] solveLinearSystem(double[][] A, double[] b) {
    int n = b.length;
    double[][] augmented = new double[n][n + 1];

    // Create augmented matrix
    for (int i = 0; i < n; i++) {
      for (int j = 0; j < n; j++) {
        augmented[i][j] = A[i][j];
      }
      augmented[i][n] = b[i];
    }

    // Forward elimination
    for (int i = 0; i < n; i++) {
      // Find pivot
      int maxRow = i;
      for (int k = i + 1; k < n; k++) {
        if (Math.abs(augmented[k][i]) > Math.abs(augmented[maxRow][i])) {
          maxRow = k;
        }
      }

      // Swap rows
      double[] temp = augmented[i];
      augmented[i] = augmented[maxRow];
      augmented[maxRow] = temp;

      // Eliminate column
      for (int k = i + 1; k < n; k++) {
        double factor = augmented[k][i] / augmented[i][i];
        for (int j = i; j <= n; j++) {
          augmented[k][j] -= factor * augmented[i][j];
        }
      }
    }

    // Back substitution
    double[] solution = new double[n];
    for (int i = n - 1; i >= 0; i--) {
      double sum = 0;
      for (int j = i + 1; j < n; j++) {
        sum += augmented[i][j] * solution[j];
      }
      solution[i] = (augmented[i][n] - sum) / augmented[i][i];
    }

    return solution;
  }
}