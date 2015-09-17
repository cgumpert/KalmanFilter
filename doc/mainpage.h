/**
   \mainpage Kalman Filter

   \tableofcontents

   \section installation Installation/Usage Remarks

   This package depends on the <a href="http://eigen.tuxfamily.org/index.php?title=Main_Page">Eigen</a> library for the implementation of the matrix operations.
   
   \section intro Introduction to Kalman Filters

   \subsection Notation

   A Kalman filter is an algorithm which is able to infer the inner state of a stochastic system from a set
   of possibly noisy measurements. The system evolution must be expressible in a linear discretized form as

   \f[ \mathbf{x}_k = \mathbf{A}_{k-1} \mathbf{x}_{k-1} + \mathbf{B}_{k-1} \mathbf{u}_{k-1} + \mathbf{w}_{k-1} \f]
   
   where:

   - \f$\mathbf{x}_k \in \mathbb{R}^n\f$ describes the inner state of the system at step \f$k\f$,
   - \f$\mathbf{A}_k \in \mathbb{R}^{n \times n}\f$ describes the propagation of the inner state from step \f$k\f$ to \f$k+1\f$,
   - \f$\mathbf{u}_k \in \mathbb{R}^n\f$ represents external control inputs at step \f$k\f$,
   - \f$\mathbf{B}_k \in \mathbb{R}^{n \times n}\f$ encodes how the control inputs affect the evolution of the internal state from step \f$k\f$ to \f$k+1\f$,
   - \f$\mathbf{w}_k \in \mathbb{R}^n\f$ is the process noise which is assumed to be normal distributed with a mean of zero and a covariance \f$\mathbf{Q}_k \in \mathbb{R}^{n \times n}\f$.

   Furthermore, the measurement at step \f$k\f$ depends linearly on the internal state of the system and is affected by some uncertainty.
   It can therefore be described as

   \f[ \mathbf{z}_k = \mathbf{H}_k \mathbf{x}_k + \mathbf{v}_k \f]

   with:

   - \f$\mathbf{z}_k \in \mathbb{R}^m\f$ being the \f$m\f$-dimensional measurement at step \f$k\f$,
   - \f$\mathbf{H}_k \in \mathbb{R}^{m \times n}\f$ being the projection matrix which relates the space of the internal state at step \f$k\f$ to the space of the corresponding measurement,
   - \f$\mathbf{v}_k \in \mathbb{R}^m\f$ representing the measurement noise(uncertainty) which is assumed to be normal distributed around a mean of zero (i.e. the measurements should be unbiased)
     and with a covariance \f$\mathbf{R}_k \in \mathbb{R}^{m \times m}\f$.

   The <b>goal</b> of the Kalman Filter algorithm is to estimate the internal state \f$\mathbf{x}_k\f$ of the system at each step \f$k\f$ and provide a covariance matrix \f$\mathbf{P}_k\f$ for this estimate.
   To this end the following notation is introduced. The estimate for the system's internal state at step \f$k\f$, but before taking into account the measurement \f$\mathbf{z}_k\f$, is denoted as
   \f$\mathbf{\hat{x}}_{k-1}^k\f$. Its covariance matrix shall be denoted by \f$\mathbf{P}_{k-1}^k\f$. The estimate of the state after considering the measurement \f$\mathbf{z}_k\f$ is labelled
   \f$\mathbf{\hat{x}}_k^k\f$ with the corresponding covariance matrix \f$\mathbf{P}_k^k\f$. Furthermore, a primed matrix indicates the transposed matrix, \f$\mathbf{M}^\prime \equiv \mathbf{M}^\mathrm{T}\f$.

   \subsection equations State Evolution, Update and Smoothing Equations

   The internal state of the system and its associated covariance matrix can be <b><i>predicted</i></b> from step \f$k-1\f$ to the step \f$k\f$ using the two equations given below. Since the
   stochastic process noise \f$\mathbf{w}_{k-1}\f$ is by definition not known, the prediction of the internal state representation employs the assumption that the process noise has a zero mean.
   Of course, the covariance of the process noise contributes to the predicted covariance of the extrapolated state vector.

   \f{eqnarray*}{
   \mathbf{\hat{x}}_{k-1}^k &=& \mathbf{A}_{k-1} \mathbf{\hat{x}}_{k-1}^{k-1} + \mathbf{B}_{k-1} \mathbf{u}_{k-1} , \\
   \mathbf{P}_{k-1}^k &=& \mathbf{A}_{k-1} \mathbf{P}_{k-1}^{k-1} \mathbf{A}_{k-1}^\prime + \mathbf{Q}_{k-1} .
   \f}

   This prediction can then be <b><i>updated</i></b> with the information from the measurement \f$\mathbf{z}_k\f$ using:
   
   \f{eqnarray*}{
   \mathbf{\hat{x}}_k^k &=& \mathbf{\hat{x}}_{k-1}^k + \mathbf{K}_k \left( \mathbf{z}_k - \mathbf{H}_k \mathbf{\hat{x}}_{k-1}^k \right) , \\
   \mathbf{P}_k^k &=& \left( \mathbf{1}_{n \times n} - \mathbf{K}_k \mathbf{H}_k \right) \mathbf{P}_{k-1}^k \\
   \text{with} \qquad \mathbf{K}_k &=& \mathbf{P}_{k-1}^k \mathbf{H}_k^\prime \left( \mathbf{H}_k \mathbf{P}_{k-1}^k \mathbf{H}_k^\prime + \mathbf{R}_k \right)^{-1} .
   \f}

   Using the information from all measurements, one can perform a backward extrapolation which <b><i>smoothes</i></b> the estimates of the internal state and its covariance at all previous steps.
   Let \f$\mathbf{\hat{x}}_T^k\f$ denote the estimate for the internal state at step \f$k\f$ using all measurements \f$\{\mathbf{z}_1, \dots , \mathbf{z}_T\}\f$ and let \f$\mathbf{P}_T^k\f$ be the associated covariance matrix.
   Start with computing \f$\mathbf{\hat{x}}_T^T\f$ and \f$\mathbf{P}_T^T\f$ using the prediction and update step given above. Then the following smoothing equations can be applied:

   \f{eqnarray*}{
   \mathbf{\hat{x}}_T^k &=& \mathbf{\hat{x}}_k^k + \mathbf{G}_k \left(\mathbf{\hat{x}}_T^{k+1} - \mathbf{\hat{x}}_k^{k+1} \right) , \\
   \mathbf{P}_T^k &=& \mathbf{P}_k^k + \mathbf{G}_k \left( \mathbf{P}_T^{k+1} - \mathbf{P}_k^{k+1} \right) \mathbf{G}_k^\prime , \\
   \text{with} \qquad \mathbf{G}_k &=& \mathbf{P}_k^k \mathbf{A}_k^\prime \left( \mathbf{P}_k^{k+1} \right)^{-1} .
   \f}

   \subsection EKF Extension to non-linear processes and measurements - Extended Kalman Filter (EKF)

   So far, the system evolution equation and the measurement mapping were assumed to be linear which severly limits the number of problems where Kalman Filters can be applied.
   However, one can extend the concept to non-linear systems/measurement by performing a linear approximation. Let the state evoluation and the measurement mapping be given by

   \f{eqnarray*}{
   \mathbf{x}_k &=& f(\mathbf{x}_{k-1},\mathbf{u}_{k-1},\mathbf{w}_{k-1}) , \\
   \mathbf{z}_k &=& h(\mathbf{x}_k,\mathbf{v}_k) .
   \f}
   
   where both, the <i>state propagation</i> \f$f\f$ and the <i>measurement mapping</i> \f$h\f$, might be non-linear functions. As the process noise and measurement noise are not known, but are assumed to have a zero mean, one can approximate
   the true values by the following Taylor expansion to first order

   \f{eqnarray*}{
   \mathbf{x}_k &\approx& f(\mathbf{\hat{x}}_{k-1}^{k-1},\mathbf{u}_{k-1},0) + \mathbf{A}_{k-1} \left(\mathbf{x}_{k-1} - \mathbf{\hat{x}}_{k-1}^{k-1} \right) + \mathbf{W}_{k-1} \mathbf{w}_{k-1} , \\
   \mathbf{z}_k &\approx& h(\mathbf{\hat{x}}_k^k,0) + \mathbf{H}_k \left( \mathbf{x}_k - \mathbf{\hat{x}}_k^k \right) + \mathbf{V}_k \mathbf{v}_k
   \f}

   where the matrices are now defined as Jacobians as

   \f{eqnarray*}{
    \left( \mathbf{A}_k \right)_{ij} &=& \left. \frac{\partial f_i}{\partial x_j} \right|_{\left( \mathbf{\hat{x}}_k^k,\mathbf{u}_k,0 \right)} \in \mathbb{R}^{n \times n} , \\
    \left( \mathbf{W}_k \right)_{ij} &=& \left. \frac{\partial f_i}{\partial w_j} \right|_{\left( \mathbf{\hat{x}}_k^k,\mathbf{u}_k,0 \right)} \in \mathbb{R}^{n \times n} , \\
    \left( \mathbf{H}_k \right)_{ij} &=& \left. \frac{\partial h_i}{\partial x_j} \right|_{\left( \mathbf{\hat{x}}_k^k,0 \right)} \in \mathbb{R}^{m \times n}, \\
    \left( \mathbf{V}_k \right)_{ij} &=& \left. \frac{\partial h_i}{\partial v_j} \right|_{\left( \mathbf{\hat{x}}_k^k,0 \right)} \in \mathbb{R}^{m \times m} .
   \f}

   Under this approximation and with this notation, the prediction and update equations become:
   
   \f{eqnarray}{
   \mathbf{\hat{x}}_{k-1}^k &=& f(\mathbf{\hat{x}}_{k-1}^{k-1},\mathbf{u}_{k-1},0) , \\
   \mathbf{P}_{k-1}^k &=& \mathbf{A}_{k-1} \mathbf{P}_{k-1}^{k-1} \mathbf{A}_{k-1}^\prime + \mathbf{W}_{k-1} \mathbf{Q}_{k-1} \mathbf{W}_{k-1}^\prime , \\
   \mathbf{\hat{x}}_k^k &=& \mathbf{\hat{x}}_{k-1}^k + \mathbf{K}_k \left( \mathbf{z}_k - h(\mathbf{\hat{x}}_{k-1}^k,0) \right) , \\
   \mathbf{P}_k^k &=& \left( \mathbf{1}_{n \times n} - \mathbf{K}_k \mathbf{H}_k \right) \mathbf{P}_{k-1}^k \\
   \text{with} \qquad \mathbf{K}_k &=& \mathbf{P}_{k-1}^k \mathbf{H}_k^\prime \left( \mathbf{H}_k \mathbf{P}_{k-1}^k \mathbf{H}_k^\prime + \mathbf{V}_k \mathbf{R}_k \mathbf{V}_k^\prime \right)^{-1} .
   \f}

   The form of the smoothing equations remains unchanged with the difference that \f$\mathbf{A}_k\f$ now stands for the Jacobian of the state extrapolation function with respect to \f$\mathbf{x}_k\f$.
   
   \section impl Implementation

   In order to implement the extended Kalman Filter using the equations outlined above, one needs to provide the following entities:

   -# a representation of the <i>state vector</i> \f$\mathbf{x}_k\f$ (which should be the same for any \f$k\f$),
   -# a representation of the <i>covariance matrix</i> \f$\mathbf{P}_k\f$ of the internal state (which should be the same for any \f$k\f$),
   -# the state propagation function (= the underlying stochastical model) \f$\mathbf{x}_{k+1} = f\left(\mathbf{x}_k, \mathbf{u}_k, \mathbf{w}_k \right)\f$ for all allowed values of \f$k\f$,
   -# the measurement mapping function \f$\mathbf{z}_k = h\left(\mathbf{x}_k, \mathbf{v}_k \right)\f$ for all allowed values of \f$k\f$,
   -# the covariance of the process noise, \f$\mathbf{Q}_k\f$, at each step \f$k\f$,
   -# the covariance of the measurement noise, \f$\mathbf{R}_k\f$, at each step \f$k\f$,
   -# the jacobian of the state propagation function with respect to the state vector, \f$\mathbf{A}_k\f$, at each step \f$k\f$,
   -# the jacobian of the state propagation function with respect to the process noise vector, \f$\mathbf{W}_k\f$, at each step \f$k\f$,
   -# the jacobian of the measurement mapping function with respect to the state vector, \f$\mathbf{H}_k\f$, at each step \f$k\f$,
   -# the jacobian of the measurement mapping function with respect to the measurement noise vector, \f$\mathbf{V}_k\f$, at each step \f$k\f$.

   A KF::BaseState class is provided as pure virtual base class for specific implementations of the internal state of the system. This class provides an interface for accessing the
   state vector \f$\mathbf{x}_k\f$ as well as its covariance matrix \f$\mathbf{P}_k\f$. The system evoluation \f$f\f$ is modelled by the KF::BasePredictor class which has an interface for propagating
   the system state \f$\mathbf{x}_{k+1} = f(\mathbf{x}_k, \dots)\f$ and its covariance matrix. In addition, it has a pure virtual private interface for obtaining the process noise \f$\mathbf{Q}_k\f$
   as well as the jacobian matrices \f$\mathbf{A}_k\f$ and \f$\mathbf{W}_k\f$. The abstract KF::CompatibleMeasurement class provides the interface for performing the update step of a state using the
   information from a measurement. This interface only depends on the type of the system state, most notably on its dimensionality \f$n\f$. Since measurements of different dimensionality \f$m\f$
   can constrain the internal state of the system, another abstraction layer is introduced. The abstract KF::BaseMeasurement class is derived from KF::CompatibleMeasurement and implements the state
   update equation for a specific dimension \f$m\f$ of the measurement. Thereby, it relies on an abstract public interface for accessing the actual measurement \f$\mathbf{z}_k\f$ and its covariance
   matrix \f$\mathbf{R}_k\f$ as well as on a private pure virtual interface for the measurement mapping function \f$h\f$ and the accessors to the jacobian matrices \f$\mathbf{H}_k\f$ and \f$\mathbf{V}_k\f$.

   One subtlety which is not directly obvious from the state evolution equation (1) above, is the fact that the prediction of the system's internal state may depend on the next measurement one wants to
   compare to. That is, the step index \f$k\f$ on its own is not meaningful, but only in conjunction with the related measurement. As an example one could imagine the propagation of a particle in the
   two-dimensional \f$(x - y)\f$-plane. The \f$y\f$ position of the particle is measured by devices located at certain positions in \f$x\f$. If the distance \f$\Delta x\f$ between the measurement
   devices is not constant, one needs to pass the \f$x\f$ position of the next device to the algorithm which propagates the position of the particle. Otherwise, no meaningful comparison between the predicted
   particle position and the actual measurement is possible. This coupling creates a dependency between measurement and the system evolution which is not apparent in the above equations. Unfortunately,
   the prediction algorithm may need to have access to the <i>actual</i> measurement type for each propagation step. This behaviour is achieved using a double dispatch mechanism via the
   <a href="https://en.wikipedia.org/wiki/Visitor_pattern">visitor pattern</a>. As a consequence (and maybe a bit counter-intuitive), the prediction of state to the next measurement is achieved by a call to
   KF::CompatibleMeasurement::acceptPredictor.
   
   The Kalman Filter can be invoked on a list of KF::CompatibleMeasurement's, an initial state and an object for the state prediction using KF::KalmanFilter::filter. The predicted and filtered state at
   each step are cached together with the jacobian \f$\mathbf{A}_k\f$ which allows to re-use this information during the smoothing step. The smoothing is invoked by calling KF::KalmanFilter::smooth taking
   a stack of caches as input. In the course of the execution, the caches will be updated with the information from the "smoothed" state.
   
   \section ref References

   -# <a href="http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf">Welch, Bishop -- An Introduction to Kalman Filter</a>
   -# <a href="http://mplab.ucsd.edu/tutorials/Kalman.pdf">Movellan -- Discrete Time Kalman Filters and Smoothers</a>
*/

