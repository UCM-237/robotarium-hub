from sklearn.linear_model import LinearRegression #pip install scikit-learn
import numpy as np
import matplotlib.pyplot as plt

class LinearRegressionModel:
    def __init__(self):
        self.model = LinearRegression()
        self.slope = None
        self.intercept = None
        self.linearRegressionData = {}

    def train(self):
        X = np.array(list(self.linearRegressionData.keys())).reshape(-1, 1)
        y = np.array(list(self.linearRegressionData.values()))
        self.model.fit(X, y)
        self.slope = self.model.coef_[0]
        self.intercept = self.model.intercept_

    def get_line_equation(self):
        if self.slope is not None and self.intercept is not None:
            return f"y = {self.slope}x + {self.intercept}"
        else:
            return "The model has not been trained yet."

    def predict(self, X):
        return self.model.predict(X)
    def addData(self, x, y):
        self.linearRegressionData[x] = y
        #self.train(np.array(list(self.linearRegressionData.keys())).reshape(-1, 1), np.array(list(self.linearRegressionData.values())))

# # Example usage
# X = np.array([1, 2, 3, 4, 5]).reshape(-1, 1)  # Independent variables
# y = np.array([2, 3, 4, 5, 6])                 # Dependent variable

# # Create an instance of the LinearRegressionModel class
# linear_regression_model = LinearRegressionModel()

# # Train the model
# linear_regression_model.train(X, y)

# # Get the equation of the line
# line_equation = linear_regression_model.get_line_equation()
# print("Line equation:")
# print(line_equation)

# # Make predictions
# predictions = linear_regression_model.predict(X)

# # Plotting
# plt.scatter(X, y, color='blue', label='Data Points')
# plt.plot(X, predictions, color='red', label='Regression Line')
# plt.xlabel('X')
# plt.ylabel('y')
# plt.title('Linear Regression')
# plt.legend()
# plt.grid(True)
# plt.show()
