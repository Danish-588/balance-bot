# A Robust Cascaded PID Control for a Two-Wheeled Self-Balancing Robot

## Abstract

This paper presents a cascaded PID control architecture for a two-wheeled self-balancing robot—a project that stems from my longstanding interest in robotics. The system uses two interconnected PID loops: an inner loop that regulates the robot’s velocity based on its current tilt, and an outer loop that determines the necessary tilt to achieve a desired speed. To improve state estimation, a Kalman filter is integrated with data from an MPU6000-based IMU, enhancing sensor fusion and reducing noise effects. The hardware platform is built around a TI M4 microcontroller and employs DC motors with encoders along with LMD18200 motor drivers. Extensive experimental testing under varying conditions demonstrates that the dual-loop approach not only maintains balance but also supports controlled movement. This work bridges conventional PID control with advanced filtering techniques, providing a solution that balances implementation simplicity with robust dynamic performance.


## Literature Review

The literature on two-wheeled self-balancing robots covers a diverse range of control strategies, sensor fusion methods, and hardware implementations. Early work such as *Design and Control of a Two-Wheel Self-Balancing Robot Using Arduino* demonstrated that even low-cost platforms could achieve stability through the use of PID and PI-PD (LQR-based) controllers, with complementary filters effectively mitigating sensor drift citeturn0file0. Later studies like *Simulation and Control of a Two-Wheeled Self-Balancing Robot* and *Modeling and Control of a Two-Wheeled Self-Balancing Robot* compared PID controllers with LQR methods, finding that although PID is easier to implement, LQR can offer faster settling times and reduced overshoot in simulation environments citeturn0file0.

Hybrid control strategies have also been explored. For example, integrating Kalman filtering with PID control has been shown to enhance state estimation by effectively combining sensor data, which in turn improves overall system stability citeturn0file0. Other approaches, such as the use of fuzzy PD controllers, have been investigated as cost-effective alternatives for maintaining balance under uncertain conditions citeturn0file0.

Despite these advances, several gaps remain. Much of the existing work relies on static tuning methods and fixed control architectures, with limited exploration into adaptive or AI-driven techniques that could handle dynamic payloads or uneven terrain. Moreover, while traditional sensors like gyros and IMUs are commonly used, there is growing interest in incorporating advanced perception systems such as LiDAR and cameras to further enhance autonomy and performance citeturn0file0.

Motivated by these findings and challenges identified in previous studies, this research adopts a cascaded PID control strategy that leverages the simplicity of PID controllers alongside the precision of Kalman filtering. The goal is to provide a more robust and adaptable framework for real-world self-balancing robot applications—one that addresses some of the limitations observed in the literature while remaining accessible for practical implementation.

