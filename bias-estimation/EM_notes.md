## Maximum likelihood estimation

**Maximum likelihood** (ML) estimate:

$$\hat \theta \triangleq \argmax_\theta p_\theta (Y_N)$$

where $Y_N$ means $\{y_1, ..., y_N\}$; $y_t$ means the measurement at time $t$, $p$ means a probabilit density function parameterised by $\theta$.

According to the definition of conditional probabilities,

$$p_\theta (Y_n) = p_\theta (y_1) \prod_{t=2}^N p_\theta (y_t|Y_{t-1})$$

Define **log-likelihood**

$$L_\theta (Y_N) \triangleq \log p_\theta (Y_N)$$

ML estimate is equivalent to 

$$\hat \theta = \argmax_\theta L_\theta (Y)$$

> The logarithm is strictly increasing.

New ML estimate problem: To solve

$$\hat \theta = \argmax_\theta \sum_{t=2}^N L_\theta (y_t|Y_{t-1}) + L_\theta (y_1)$$

One method of solving it is **expectation maximization** (EM) algorithm.

## Expectation maximization algorithm

Difficulty: observation is incomplete => cannot solve $L_\theta (Y)$
Solution: add latent $Z$ to make it complete => to solve $L_\theta (Z,Y)$

$$L_\theta (Y) = E_{\theta_k} \{L_\theta (Z,Y)|Y\} - E_{\theta_k} \{L_\theta (Z|Y)|Y\}
\triangleq Q(\theta,\theta_k) - V(\theta,\theta_k)$$

where $\theta_k$ means the estimate of the parameter $\theta$ from the $k$-th iteration of the algorithm.

From $V(\theta,\theta_k) \le V(\theta_k,\theta_k)$ get

$$Q(\theta,\theta_k) \ge Q(\theta_k,\theta_k) \to L_\theta(Y) \ge L_{\theta_k} (Y)$$

If choose $\theta_{k+1}$ as $\argmax_\theta Q(\theta,\theta_k)$, we can get $L_{\theta_{k+1}} (Y) \ge L_{\theta_k} (Y)$. 

> \# EM algorithm
> *set $\theta_0$*
> for k in range(max_iterations):
> &emsp;*E step: compute $Q_{\theta,\theta_k}$*
> &emsp;*M step: compute $\theta_{k+1}$*
> &emsp;if *converge?*:
> &emsp;&emsp;break

How to check converged?

- Check 
$$ |L_{\theta_{k+1}} (Y) - L_{\theta_k} (Y)| \le \varepsilon_L$$
where $\varepsilon_L$ can be chosen as `1e-6`.
- Check
$$ ||\theta_{k+1} - \theta_k||^2 \le \varepsilon_p$$
where $\varepsilon_p$ is positive.