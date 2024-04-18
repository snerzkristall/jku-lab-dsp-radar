import scipy.signal as signal

n_coeffs = 64

f_cutoff = 3400

f_sample = 48000

coeffs = signal.firwin(n_coeffs, f_cutoff, window='hamming', pass_zero='lowpass', fs=fs)

print(coeffs)