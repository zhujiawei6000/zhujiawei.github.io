---
layout: post
title:  "Kalman Filter Library implemented with C++ "
date:   2024-03-03 11:58:00
categories: C++
brief: "Kalman filter is a very commonly used filter in many domain, such as tracking, signal processing, etc. Before we learn its variants like EKF, UKF, PF, we should master kalman filter in the first place. So in this post, I try to implement kalman filter by myself with C++ ..."
---

# Inroduction
Kalman filter is a very commonly used filter in many domain, such as tracking, signal processing, etc. Before we learn its variants like EKF, UKF, PF, we should master kalman filter in the first place. So in this post, I try to implement kalman filter by myself with C++. The full code is available on https://github.com/zhujiawei6000/kalman_filter. The common process of the kalman filter can be shown in the following diagram. Let's review it step by step:

1. We calculate kalman gain K with errors in both measurement and estimate, the kalman gain means how much we can trust the measurement and the predicted value. The range of K is between 0 and 1. When K is large (close to 1), we can trust more on measurements. When K is small (close to 0), we can trust more on estimates.
2. With K which we calculated in the previous step, the current measurement we get from our sensor and the previous estimate. We can calculate our best estimate for the current frame. On this step without measured value, we can also predict current estimate with previous estimate with our system model.
3. Then we calcuate our new error in the estimate using the kalman gain and the noisy we involved in the previous step.
4. Loop back to step 1. with the new error in estimate.



```mermaid
flowchart TD;
    mserror(Error in Measurement) --> K
    msv(Measured Value) --> curest
    K --> curest(2. Calculate Current Estimate)
    prvest(Previous Estimate) --> curest
    curest --> prvest
    curest --> newesterr
    K --> newesterr(3. Calculate New Error in Estimiate)
    newesterr --> esterr
    esterr(Error in Estimate) --> K(1. Calculate the Kalman Gain)
```
I will not dive too much into the provement of 6 formular of kalman filter. That's a lot of material on internet. I just focus on how to implment the filter by hand in c++ and how to design the library to fit user's need. After all the implementation of the library, there are also two more examples here to make more sense to you. Enjoying..

# Implement
## Build System
The library should depend on Eigen (c++ linear algebra library), I use Conan to manage my package and CMake to generate the files that build our project. The ```conanfile.txt``` should look like this
```
[requires]
eigen/3.4.0

[generators]
CMakeDeps
CMakeToolchain

[layout]
cmake_layout
```
And the top level ```CMakeLists.txt```, the library is header only, so I made it ```INTERFACE```.
```cmake
cmake_minimum_required(VERSION 3.15)
project(KalmanFilter CXX)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(Eigen3 REQUIRED)
add_library(${CMAKE_PROJECT_NAME} INTERFACE)
target_link_libraries(${CMAKE_PROJECT_NAME} INTERFACE Eigen3::Eigen)
target_include_directories(${CMAKE_PROJECT_NAME} INTERFACE kalman_filter)

```
## Kalman Filter Class
Kalman filter class contains two types of data,State **X** (current estimate) and covariance of State **P** (error in current estimate). The code may look like the following:
```c++
template <typename V>
using Covariance = Matrix<V::RowsAtCompileTime, V::RowsAtCompileTime>;

class KalmanFilter {
 ...

 protected:
  State X_;
  Covariance<State> P_;
};
```
Every time we estimate current state, **X** and **P** should be updated

## Prediction
The prediction is defined as a member function with kalman filter class which transit current state to the next. Math formula is shown below

For state transition:
$$ x_{k|k-1} = F_k x_{k-1|k-1} + G * u $$

For state covariance update:
$$ P_{k|k-1} = F_k P_{k-1|k-1} F_k^T + Q_k’ $$

- **F** represent state transition matrix
- **u** is the input control variable
- **G** is the matrix which maps the control variable to the state
- **Q** is the process noisy

So the code for prediction is straight-foward. We involve helper class ```SystemModel``` that wraps all the factor above. Do the predict calculation and return the predicted state.

```c++
template <typename SystemModel>
class KalmanFilter {
 public:
  template <typename Control>
  const State& Predict(const SystemModel& sys, const Control& u) {
    X_ = sys.F() * X_ + sys.G() * u;
    P_ = sys.F() * P_ * sys.F().transpose() + sys.Q();
    return GetState();
  }
  const State& GetState() const { return X_; }

 protected:
  State X_;
  Covariance<State> P_;
};
```

## Update
Another key function of kalman filter is ```Update```. Inside the function, we calculate the kalman gain **K** with measurement cov and estimate cov, and then update the state and state cov with it. The formulas are:

$$ S_k=H_k P_{k|k−1} H_k^T + R $$
$$ K_k = P_{k, k-1} H_k^T S_k^{-1} $$
$$ \vec{x}_{k,k} = \vec{x}_{k,k-1} + K_k (\vec{z}_{k} – H_k \vec{x}_{k,k-1}) $$
$$ P_{k,k} = ( I – K_k  H_k) P_{k,k-1} ( I – K_k  H_k)^{-1} + K_kRK_k^{-1}$$
where
- **K** is a Kalman Gain
- **H** is the observation matrix 
- **R** is the Measurement Uncertainty (measurement noise covariance matrix)
- **I** is an Identity Matrix (the n × n square matrix with ones on the main diagonal and zeros elsewhere)

Implementation is look like below. ```MeasurementModel``` is a wrapper class for all the measurement-related parameters. Return ```State``` class which is the best estimation of the current frame.
```c++
 
  template <typename MeasurementModel>
  const State& Update(const MeasurementModel& m,
                      const typename MeasurementModel::MeasurementType& z) {
    using Measurement = typename MeasurementModel::MeasurementType;
    Covariance<Measurement> innovation_cov =
        m.H() * P_ * m.H().transpose() + m.R();
    KalmanGain<State, Measurement> K =
        P_ * m.H().transpose() * innovation_cov.inverse();
    X_ += K * (z - m.H() * X_);
    auto I = Covariance<State>::Identity();
    Covariance<State> factor = I - K * m.H();
    P_ = factor * P_ * factor.transpose() + K * m.R() * K.transpose();
    return GetState();
  }

```

## System Model
The system model is the base class, just define some data we used in ```KalmanFilter``` class. User can extend the ```SystemModel``` class to specify its unique **F**, **Q** and **G**.
```c++

template <typename State, typename Control = Vector<0>>
struct SystemModel {
  using StateType = State;
  using ControlType = Control;
  const auto& F() const { return F_; }
  const auto& Q() const { return Q_; }
  const auto& G() const { return G_; }

 protected:
  Transition<State> F_;
  Covariance<State> Q_;
  InputTransition<State, Control> G_;
};
template <typename SystemModel>
inline constexpr bool HasControl =
    std::is_same<SystemModel::ControlType, Vector<0>>::value;

```

This is the snipper code in one of my exmple. The beauty of this design is that we change only we need change, and the library will take care of the rest.
```c++
class VehicleLocationSystemModel
    : public kf::SystemModel<VehicleLocationState> {
 public:
  using S = VehicleLocationState;
  explicit VehicleLocationSystemModel(double acc_var) : acc_var_{acc_var} {}
  void Update(double dt) {
    F_.setIdentity();
    F_(S::POS_X, S::VEL_X) = dt;
    F_(S::POS_X, S::ACC_X) = 0.5 * dt * dt;
    F_(S::VEL_X, S::ACC_X) = dt;
    F_(S::POS_Y, S::VEL_Y) = dt;
    F_(S::POS_Y, S::ACC_Y) = 0.5 * dt * dt;
    F_(S::VEL_Y, S::ACC_Y) = dt;
    Q_.setIdentity();
    Q_.block(0, 0, 3, 3) << std::pow(dt, 4) / 4, std::pow(dt, 3) / 2,
        std::pow(dt, 2) / 2, std::pow(dt, 3) / 2, std::pow(dt, 2), dt,
        std::pow(dt, 2) / 2, dt, 1;
    Q_.block(3, 3, 3, 3) << std::pow(dt, 4) / 4, std::pow(dt, 3) / 2,
        std::pow(dt, 2) / 2, std::pow(dt, 3) / 2, std::pow(dt, 2), dt,
        std::pow(dt, 2) / 2, dt, 1;
    Q_ *= acc_var_;
  }

 private:
  double acc_var_;
};
```

## Measurement Model
The measurement model is the same stuff as the SystemModel. 

```c++
template <typename State, typename Measurement>
struct MeasurementModel {
  using MeasurementType = Measurement;
  Observation<Measurement, State> H() const { return H_; }
  Covariance<Measurement> R() const { return R_; }

 protected:
  Observation<Measurement, State> H_;
  Covariance<Measurement> R_;
};
```

# Put all the thing together
With all the component is implmented, we are ready to put all the thing together. First thing first, we need to define your own ```SystemModel``` and ```MeasurementModel``` according to your problem. After your definition, you can write down the main logic of the filter process. 

1. Get the measurement off the input queue
2. Update system model with measurements' delta time
3. Predict current state condition on the previous
4. Correct the filter with current measurement, and get the best estimation of current. Repeat.

```c++
  while (!measurements.empty()) {
    auto measure = measurements.front();
    measurements.pop();
    // update system model with dt=1.0s
    system_model.Update(1.0f);
    // predict N+1 condition on N
    filter.Predict(system_model);
    // estimate N+1 with both measure & prediction
    auto state = filter.Update(measurement_model, measure);
    // std::cout << state << std::endl;
    std::cout << measure.x() << "," << measure.y() << ","
              << state.x() << "," << state.y() << ","
              << state.vx() << "," << state.vy() << std::endl;
  }
```

# Summary
At this point, I have finished implementation of the library and give you a good start. In the future I'll try to implement the extended kalman filter in C++. See you next time...
