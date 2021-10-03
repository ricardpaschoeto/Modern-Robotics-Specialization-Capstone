type of controller => feedforward-plus-PI
feedback gains => kp = 5.0*np.eye(6) and ki = 0.25*np.eye(6)
new block config:
- initial = [1.0, 0.5, 0.464]
- final = [0.5, -1., 0, -1.1]
The system converges to zero in approx. 2.75s, we have the highest overshoot at wy component.