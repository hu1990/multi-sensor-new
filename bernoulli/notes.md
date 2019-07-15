# Bernoulli filter

`state: Bernoulli RFS`
`observation: RFS`

FISST PDF of Bernoulli RFS
$$f(X) = 
\begin{cases}
1-q & X=\varnothing \\
q p(x) & X=\{x\}
\end{cases}$$

`Bernoulli RFS(q, p)`

## Basic equation

`state: Bernoulli RFS(q, s)`

### Predict

$$q_{k|k-1} = p_b (1 - q_{k-1|k-1}) + p_s q_{k-1|k-1}$$
$$s_{k|k-1}(x) = \frac{p_b (1 - q_{k-1|k-1}) b_{k|k-1}(x)}{q_{k|k-1}} + 
\frac{p_s q_{k-1|k-1} \int \pi_{k|k-1}(x|x') s_{k-1|k-1} dx'}{q_{k|k-1}}$$

where
birth probability $p_b$
survive probability $p_s$
trasmission probability $\pi_{k|k-1}(x|x')$
birth model
$$b_{k|k-1}$$

### Update

$$q_{k|k} = \dfrac
{(1 - p_d) + p_d \sum\limits_{z\in Z_k} \dfrac{\int g_k(z|x) s_{k|k-1}(x) dx}{\lambda c(z)}}
{\dfrac{1 - q_{k|k-1}}{q_{k|k-1}}+
{(1 - p_d) + p_d \sum\limits_{z\in Z_k} \dfrac{\int g_k(z|x) s_{k|k-1}(x) dx}{\lambda c(z)}}}$$

$$s_{k|k}(x) = \dfrac
{(1 - p_d) + p_d \sum\limits_{z\in Z_k} \dfrac{g_k(z|x)}{\lambda c(z)}}
{(1 - p_d) + p_d \sum\limits_{z\in Z_k} \dfrac{\int g_k(z|x) s_{k|k-1}(x) dx}{\lambda c(z)}} 
s_{k|k-1}(x)$$

where
detect probability $p_d$
cluster distribution
likelihood $g_k(z|x{})$

## Gaussian Sum implementation

### Assumption

Gaussian linear assumption

`state: Bernoulli RFS(q, s)`
`p: Gaussian Mixture(w[J],m[J],p[J],J)`

trasmission probability $\pi_{k|k-1}(x|x') = N(x; F_{k-1}x', Q_{k-1})$
likelihood $g_k(z|x{}) = N(z; H_{k-1}x, R_{k})$

### Predict

$$q_{k|k-1} = p_b (1 - q_{k-1|k-1}) + p_s q_{k-1|k-1}$$
$$s_{k|k-1}(x) = \frac{p_b (1 - q_{k-1|k-1}) b_{k|k-1}(x)}{q_{k|k-1}} + 
\frac{p_s q_{k-1|k-1}}{q_{k|k-1}}\sum\limits_{i=1}^{J_{k-1}} w_{k-1}^{(i)}N(x; m_{k|k-1}^{(i)}, {P_{k|k-1}^{(i)}})$$

where
birth probability $p_b$
survive probability $p_s$
Kalman predict
$$m_{k|k-1}^{(i)} = F_{k-1} m_{k-1}^{(i)}$$
$$P_{k|k-1}^{(i)} = F_{k-1} P_{k-1}^{(i)} F_{k-1}^T + Q_{k-1}$$
trasmission probability $\pi_{k|k-1}(x|x')$
birth model
$$b_{k|k-1} = \sum\limits_{i=1}^{J_{R,k}}w_{R,k}^{(i)}N(x;x_{R,k}^{(i)}, Q_{R,k}^{(i)})$$

### Update

$$q_{k|k} = \dfrac
{1 - \Delta_k}
{1 - q_{k|k-1} \Delta_k}
{q_{k|k-1}}
$$

$$s_{k|k}(x) = \dfrac
{(1 - p_d)s_{k|k-1}(x) + p_d \sum\limits_{z\in Z_k} \sum\limits_{i=1}^{J_{k|k-1}}\dfrac{w_{k|k-1}^{(i)}q_k^{(i)}(z) N(x; m_{k|k}^{(i)}, P_{k|k}^{(i)})}{\lambda c(z)}}
{1 - \Delta_k}
$$

$$
\Delta_k = { p_d(1 - \sum\limits_{z\in Z_k} \sum\limits_{i=1}^{J_{k|k-1}}\dfrac{w_{k|k-1}^{(i)}q_k^{(i)}(z)}{\lambda c(z)}})
$$
where
detect probability $p_d$
cluster distribution
Kalman update
$$q_k^{(i)}(z) = N(z; \eta_{k|k-1}^{(i)}, S_{k|k-1}^{(i)})$$
$$\eta_{k|k-1}^{(i)} = H_{k} m_{k|k-1}^{(i)}$$
$$S_{k|k-1}^{(i)} = H_k P_{k|k-1}^{(i)} H_k^T+ R_k$$
$$m_{k|k}^{(i)} = m_{k|k-1}^{(i)} + K_k^{(i)}(z - \eta_{k|k-1}^{(i)})$$
$$P_{k|k}^{(i)} = (I - K_k^{(i)} H_k) P_{k|k}^{(i)}$$
$$K_k^{(i)} = P_{k|k-1}^{(i)} H_k^T [S_{k|k-1}^{(i)}]^{-1}$$