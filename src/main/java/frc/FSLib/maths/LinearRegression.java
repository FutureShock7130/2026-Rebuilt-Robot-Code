package frc.FSLib.maths;

// This class is generate by Sonnet-4.5 and verified by Wjxcws
public class LinearRegression {
  private double slope;
  private double intercept;
  private double rSquared;

  public LinearRegression(double[] x, double y[]) {
    this.slope = 0;
    this.intercept = 0;
    this.rSquared = 0;
    fit(x, y);
  }

  // Fit the linear model to the data
  private void fit(double[] x, double[] y) {
    if (x.length != y.length) {
      throw new IllegalArgumentException("x and y arrays must have the same length");
    }

    if (x.length < 2) {
      throw new IllegalArgumentException("Need at least 2 data points");
    }

    int n = x.length;

    // Calculate means
    double sumX = 0, sumY = 0;
    for (int i = 0; i < n; i++) {
      sumX += x[i];
      sumY += y[i];
    }
    double meanX = sumX / n;
    double meanY = sumY / n;

    // Calculate slope and intercept
    double numerator = 0;
    double denominator = 0;

    for (int i = 0; i < n; i++) {
      numerator += (x[i] - meanX) * (y[i] - meanY);
      denominator += (x[i] - meanX) * (x[i] - meanX);
    }

    slope = numerator / denominator;
    intercept = meanY - slope * meanX;

    // Calculate R-squared
    double ssTotal = 0;
    double ssResidual = 0;

    for (int i = 0; i < n; i++) {
      double predicted = predict(x[i]);
      ssTotal += (y[i] - meanY) * (y[i] - meanY);
      ssResidual += (y[i] - predicted) * (y[i] - predicted);
    }

    rSquared = 1 - (ssResidual / ssTotal);
  }

  // Predict y value for a given x
  public double predict(double x) {
    return slope * x + intercept;
  }

  // Get the slope
  public double getSlope() {
    return slope;
  }

  // Get the intercept
  public double getIntercept() {
    return intercept;
  }

  // Get R-squared value (goodness of fit)
  public double getRSquared() {
    return rSquared;
  }

  // Get the equation as a string
  public String getEquation() {
    if (intercept >= 0) {
      return String.format("y = %.4fx + %.4f", slope, intercept);
    } else {
      return String.format("y = %.4fx - %.4f", slope, Math.abs(intercept));
    }
  }
}