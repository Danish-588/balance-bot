## Existing Literature

### 1. **[Design and Control of a Two-Wheel Self-Balancing Robot Using Arduino](https://ieeexplore.ieee.org/abstract/document/6565146)**
   - **Summary**: This study demonstrates a low-cost two-wheel self-balancing robot using Arduino, DC motors, a gyroscope, and an accelerometer. A complementary filter compensates for gyro drift, and the system uses PID and PI-PD (LQR-based) controllers for stability. The experiment shows that the PI-PD approach is robust. Future work includes improving mechanical design and adding remote control and obstacle avoidance sensors.

---

### 2. **[Simulation and Control of a Two-Wheeled Self-Balancing Robot](https://ieeexplore.ieee.org/abstract/document/6739501)**
   - **Summary**: This paper presents the dynamic modeling and simulation of a two-wheel self-balancing robot. It compares PID and LQR control methods, with LQR shown to perform better in maintaining stability. Future work aims to validate LQR's effectiveness on a physical robot.

---

### 3. **[Modeling and Control of a Two-Wheeled Self-Balancing Robot](https://ieeexplore.ieee.org/abstract/document/6828364)**
   - **Summary**: Focuses on a two-wheeled robot modeled as an inverted pendulum system, testing PID, LQR, and hybrid controllers. Simulations reveal LQR's faster settling time with minimal overshoot, making it ideal for stability and mobility in mechatronic applications.

---

### 4. **[Two-Wheel Balancing Robot with Line Following Capability](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=92cf313fb00329a9252cb9b656e68c7c1de4ac19)**
   - **Summary**: Combines a self-balancing robot with line-following capability using an ATMEGA32 microcontroller, PID control, and infrared sensors for line detection and obstacle avoidance. Future improvements could include gyroscopic sensors for stability on uneven surfaces.

---

### 5. **[Two-Wheel Balancing Robot: Review on Control Methods and Experiments](https://www.researchgate.net/profile/Azhar-Mohd-Ibrahim/publication/332537297_Two-wheel_Balancing_Robot_Review_on_Control_Methods_and_Experiments/links/5cba6e0aa6fdcc1d49a1089d/Two-wheel-Balancing-Robot-Review-on-Control-Methods-and-Experiments.pdf)**
   - **Summary**: This review compares linear, non-linear, and self-adapting control methods for two-wheeled robots, highlighting their stability and maneuverability. The effectiveness of various sensors, such as gyros and IMUs, in maintaining balance and control is discussed. It also suggests future research should aim for more objective evaluations.

---

### 6. **[A Two-Wheeled Self-Balancing Robot with Dynamics Model](https://ieeexplore.ieee.org/abstract/document/8255365)**
   - **Summary**: Discusses a self-balancing robot using an inverted pendulum model, PID control, and Kalman filtering. The paper compares Kalman and complementary filters and suggests that while Kalman offers better performance, it is more complex. Stability depends on PID tuning, sensor fusion, and hardware optimization.

---

### 7. **[Building a Two-Wheeled Balancing Robot](https://sear.unisq.edu.au/6168/)**
   - **Summary**: Focuses on constructing a two-wheeled balancing robot using the inverted pendulum model. The paper details the design, hardware, and software implementation, followed by performance evaluation. Recommendations for improvements and future work are provided.

---

### 8. **[A Two-Wheeled Self-Balancing Robot with the Fuzzy PD Control Method](https://onlinelibrary.wiley.com/doi/full/10.1155/2012/469491)**
   - **Summary**: This study applies fuzzy PD control to a two-wheeled self-balancing robot. Using Newtonian dynamics, it adjusts controller parameters for optimal performance. The fuzzy PD controller effectively maintains balance, demonstrating cost-effective, low-cost solutions for self-balancing robots.

---

### **Comparison Table**

| **Paper Title**                                             | **Control Methods**               | **Key Findings**                                                                                                                                          | **Future Directions**                                                         |
|-------------------------------------------------------------|-----------------------------------|----------------------------------------------------------------------------------------------------------------------------------------------------------|-------------------------------------------------------------------------------|
| [Design and Control of a Two-Wheel Self-Balancing Robot](https://ieeexplore.ieee.org/abstract/document/6565146)       | PID, PI-PD (LQR-based)            | PI-PD controller is robust and achieves stability with low-cost components.                                                                               | Improve mechanical design, add remote control, and obstacle avoidance.       |
| [Simulation and Control of a Two-Wheeled Self-Balancing Robot](https://ieeexplore.ieee.org/abstract/document/6739501) | PID, LQR                          | LQR outperforms PID in maintaining stability.                                                                                                             | Apply LQR to a physical robot for validation.                                 |
| [Modeling and Control of a Two-Wheeled Self-Balancing Robot](https://ieeexplore.ieee.org/abstract/document/6828364)   | PID, LQR, Hybrid Controllers      | LQR provides better performance with minimal overshoot in simulations.                                                                                   | Enhance mobility and stability for mechatronic applications.                  |
| [Two-Wheel Balancing Robot with Line Following Capability](https://citeseerx.ist.psu.edu/document?repid=rep1&type=pdf&doi=92cf313fb00329a9252cb9b656e68c7c1de4ac19)    | PID                               | Demonstrates dynamic stabilization and effective line-following with infrared sensors.                                                                  | Add gyroscopic sensors for uneven surface balancing.                         |
| [Two-Wheel Balancing Robot: Review on Control Methods](https://www.researchgate.net/profile/Azhar-Mohd-Ibrahim/publication/332537297_Two-wheel_Balancing_Robot_Review_on_Control_Methods_and_Experiments/links/5cba6e0aa6fdcc1d49a1089d/Two-wheel-Balancing-Robot-Review-on-Control-Methods-and-Experiments.pdf)        | Linear, Non-linear, Self-adapting  | Evaluates control methods for stability and maneuverability. Sensors like gyros and IMUs are essential.                                                  | More objective evaluations of control methods.                               |
| [A Two-Wheeled Self-Balancing Robot with Dynamics Model](https://ieeexplore.ieee.org/abstract/document/8255365)       | PID, Kalman Filtering             | Kalman filter outperforms complementary filters for stability but is more complex.                                                                      | Optimize sensor fusion and PID tuning.                                       |
| [Building a Two-Wheeled Balancing Robot](https://sear.unisq.edu.au/6168/)                      | Inverted Pendulum Model           | Details the design, hardware, and software implementation with performance evaluation.                                                                   | Focus on performance improvement and future research.                         |
| [A Two-Wheeled Self-Balancing Robot with the Fuzzy PD Control Method](https://onlinelibrary.wiley.com/doi/full/10.1155/2012/469491)     | Fuzzy PD                          | Fuzzy PD control effectively maintains balance and enhances system performance.                                                                          | Focus on cost-effective solutions with low-cost components.                  |



# Gaps

Focus on traditional control methods like PID and LQR, with limited use of adaptive or AI-driven techniques.

Lack of solutions for dynamic adaptability in uneven terrain or varying payload conditions.

Minimal integration of advanced sensor technologies like LiDAR or cameras for improved perception.

Absence of standardized benchmarks for performance evaluation and comparison.

Emphasis on single-task functionality, with limited exploration of multi-tasking or autonomous navigation.

Insufficient focus on balancing cost and performance for scalable, real-world applications.



