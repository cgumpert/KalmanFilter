/**
   \mainpage Kalman Filter

   \section intro Introduction and Notation

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

   The following notation is introduced. The estimate for the system's internal state at step \f$k\f$, but before taking into account the measurement
   \f$\mathbf{z}_k\f$, is denoted as \f$\mathbf{\hat{x}}_{k-1}^k\f$. Its covariance matrix shall be denoted by \f$\mathbf{P}_{k-1}^k\f$. The estimate of the state after considering the measurement
   \f$\mathbf{z}_k\f$ is labelled \f$\mathbf{\hat{x}}_k^k\f$ with the corresponding covariance matrix \f$\mathbf{P}_k^k\f$. Furthermore, a primed matrix indicates the transposed matrix, \f$\mathbf{M}^\prime \equiv \mathbf{M}^\mathrm{T}\f$.

   \section equations State Evolution, Update and Smoothing Equations

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

   \section EKF Extension to non-linear processes and measurements - Extended Kalman Filter (EKF)

   So far, the system evolution equation and the measurement mapping were assumed to be linear which severly limits the number of problems where Kalman Filters can be applied.
   However, one can extend the concept to non-linear systems/measurement by performing a linear approximation. Let the state evoluation and the measurement mapping be given by

   \f{eqnarray*}{
   \mathbf{x}_k &=& f(\mathbf{x}_{k-1},\mathbf{u}_{k-1},\mathbf{w}_{k-1}) , \\
   \mathbf{z}_k &=& h(\mathbf{x}_k,\mathbf{v}_k) .
   \f}
   
   where both \f$f\f$ and \f$h\f$ might be non-linear functions. As the process noise and measurement noise are not known, but are assumed to have a zero mean, one can approximate
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
    \left( \mathbf{V}_k \right)_{ij} &=& \left. \frac{\partial h_i}{\partial v_j} \right|_{\left( \mathbf{\hat{x}}_k^k,0 \right)} \in \mathbb{R}^{m \times m}.
   \f}

   Under this approximation and with this notation, the prediction and update equations become:
   
   \f{eqnarray*}{
   \mathbf{\hat{x}}_{k-1}^k &=& f(\mathbf{\hat{x}}_{k-1}^{k-1},\mathbf{u}_{k-1},0) , \\
   \mathbf{P}_{k-1}^k &=& \mathbf{A}_{k-1} \mathbf{P}_{k-1}^{k-1} \mathbf{A}_{k-1}^\prime + \mathbf{W}_{k-1} \mathbf{Q}_{k-1} \mathbf{W}_{k-1}^\prime , \\
   \mathbf{\hat{x}}_k^k &=& \mathbf{\hat{x}}_{k-1}^k + \mathbf{K}_k \left( \mathbf{z}_k - h(\mathbf{\hat{x}}_{k-1}^k,0) \right) , \\
   \mathbf{P}_k^k &=& \left( \mathbf{1}_{n \times n} - \mathbf{K}_k \mathbf{H}_k \right) \mathbf{P}_{k-1}^k \\
   \text{with} \qquad \mathbf{K}_k &=& \mathbf{P}_{k-1}^k \mathbf{H}_k^\prime \left( \mathbf{H}_k \mathbf{P}_{k-1}^k \mathbf{H}_k^\prime + \mathbf{V}_k \mathbf{R}_k \mathbf{V}_k^\prime \right)^{-1} .
   \f}

   \section ref References

   -# <a href="http://www.cs.unc.edu/~welch/media/pdf/kalman_intro.pdf">Welch, Bishop -- An Introduction to Kalman Filter</a>
   -# <a href="http://mplab.ucsd.edu/tutorials/Kalman.pdf">Movellan -- Discrete Time Kalman Filters and Smoothers</a>
*/

