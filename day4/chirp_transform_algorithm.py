import numpy as np
import matplotlib.pyplot as plt

# Simulation parameters

N = 1024 # signal length
M = 256 # spectrum / convolution width
f_signal = 110 # test frequency
f_s = 8000 # sampling frequency
f_start = 100 # starting frequency
omega_null = 2 * np.pi * f_start # starting frequency

UNSURE = 200 # not sure how to space correctly
delta_omega = 2 * np.pi / M * UNSURE # frequency spacing

omega = np.array([omega_null + k * delta_omega for k in range(M)])

# Coefficients calculation

W = np.exp(-1j*delta_omega)
h_1 = np.array([W**(-(n-N+1)**2 / 2) for n in range(N+M-1)])
m_1 = np.array([np.exp(-1j*omega_null*n) * (W**(n**2 / 2)) for n in range(N)])
m_1 = np.append(m_1, np.zeros(M-1)) # zeropad right-side
m_2 = np.array([W**((n-N+1)**2 / 2) for n in range(N-1, N+M-1)])
m_2 = np.append(np.zeros(N-1), m_2) # zeropad left-side

# DTFT

x = np.array([np.sin(2 * np.pi * k * f_signal/f_s) for k in range(N+M-1)]) # test signal
tmp_1 = np.multiply(x, m_1)
tmp_2 = np.convolve(tmp_1, h_1, mode='full')
y = np.multiply(tmp_2[N-1:M+N-1], m_2[N-1:]) # CTA result
X_spect = y

# Plot

plt.figure()
plt.plot(omega / (2*np.pi), X_spect)
plt.show()
# plt.safefig('CTA_sine_spectrum', dpi=1200)