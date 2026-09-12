import numpy as np
import matplotlib.pyplot as plt
from scipy.signal import butter, filtfilt, hilbert, resample
from scipy.io import wavfile
import warnings

warnings.filterwarnings("ignore", category=wavfile.WavFileWarning)

# 参数设置
upsample_factor = 100        # 过采样倍数
input_file = 'input.wav'
fc = 1000000                 # 载波频率 (Hz)
Ac = 3.3                     # 载波幅度
kf = 75000                   # 调频灵敏度 (Hz/V)
snr_dB = 10                  # 信噪比（dB）

ka = 0.5                     # 调幅系数

# 读取音频文件并处理音频数据
fs, audio = wavfile.read(input_file)
if len(audio.shape) > 1:  # 如果是立体声，取平均
    audio = np.mean(audio, axis=1)
audio = audio.astype(np.float64)
audio = audio / np.max(np.abs(audio))  # 归一化到 [-1, 1]

# 过采样处理和计算时间常量
original_length = len(audio)
upsampled_length = original_length * upsample_factor
audio_upsampled = resample(audio, upsampled_length).astype(np.float64)
fs_upsampled = fs * upsample_factor
duration_upsampled = upsampled_length / fs_upsampled
t = np.linspace(0, duration_upsampled, upsampled_length, dtype=np.float64)

# 使用升采样后的信号进行后续处理
audio = audio_upsampled

# FM 调制
c_t = Ac * np.cos(2 * np.pi * fc * t).astype(np.float64)
phi_t = 2 * np.pi * kf * np.cumsum(audio) / fs_upsampled
fm_t = Ac * np.cos(2 * np.pi * fc * t + phi_t).astype(np.float64)

# AM 调制
am_t = Ac * (1 + ka * audio) * np.cos(2 * np.pi * fc * t).astype(np.float64)

# 增加模拟噪声
def add_noise(signal, snr_db):
    signal_power = np.mean(signal**2)
    noise_power = signal_power / (10 ** (snr_db / 10))
    noisy_signal = np.random.normal(0, np.sqrt(noise_power), signal.shape).astype(np.float64) + signal
    return noisy_signal

fm_t_noisy = add_noise(fm_t, snr_dB)
am_t_noisy = add_noise(am_t, snr_dB)

# 解调 FM 信号
analytic_signal = hilbert(fm_t_noisy)
inst_phase = np.unwrap(np.angle(analytic_signal)).astype(np.float64)
inst_freq = np.diff(inst_phase) * fs_upsampled / (2 * np.pi)
inst_freq = np.append(inst_freq, inst_freq[-1])
fm_demod_signal = (inst_freq - fc) / kf

# 解调 AM 信号 (包络检波)
am_envelope = np.abs(hilbert(am_t_noisy)).astype(np.float64)
am_demod_signal = (am_envelope - Ac) / (Ac * ka)

# 低通滤波器
def butter_lowpass(cutoff, fs, order=2):
    nyq = 0.5 * fs
    normal_cutoff = cutoff / nyq
    b, a = butter(order, normal_cutoff, btype='low', analog=False)
    return b.astype(np.float64), a.astype(np.float64)

cutoff = 22100
b, a = butter_lowpass(cutoff, fs_upsampled, order=6)
fm_demod_signal_filtered = filtfilt(b, a, fm_demod_signal).astype(np.float64)
am_demod_signal_filtered = filtfilt(b, a, am_demod_signal).astype(np.float64)

# 最终信号限制在 [-1, 1]
fm_demod_signal_final = np.clip(fm_demod_signal_filtered, -1.0, 1.0)
am_demod_signal_final = np.clip(am_demod_signal_filtered, -1.0, 1.0)

# 降采样并保存为 44100 Hz 的 WAV 文件
def downsample_and_save(signal, filename):
    final_length = len(signal)
    ratio = 44100 / fs_upsampled
    demod_resampled = resample(signal, int(final_length * ratio)).astype(np.float64)
    demod_resampled = np.clip(demod_resampled, -1.0, 1.0)
    demod_resampled_int16 = (demod_resampled * 32767).astype(np.int16)
    wavfile.write(filename, 44100, demod_resampled_int16)

downsample_and_save(fm_demod_signal_final, 'output_fm.wav')
downsample_and_save(am_demod_signal_final, 'output_am.wav')

# 绘图部分
N = len(t)
idx_end = int(N * 0.01)  # 取前1%用于绘图

t_plot = t[:idx_end] * 1000  # 单位转换成毫秒
audio_plot = audio[:idx_end]
c_t_plot = c_t[:idx_end]
fm_t_noisy_plot = fm_t_noisy[:idx_end]
am_t_noisy_plot = am_t_noisy[:idx_end]
fm_demod_signal_plot = fm_demod_signal_final[:idx_end]
am_demod_signal_plot = am_demod_signal_final[:idx_end]

plt.figure(figsize=(12, 16))

# 输入信号
plt.subplot(6, 1, 1)
plt.plot(t_plot, audio_plot)
plt.title('Input Audio Signal (First 1%)')
plt.xlabel('Time (ms)')
plt.ylabel('Amplitude')
plt.grid(True)

# 载波信号
plt.subplot(6, 1, 2)
plt.plot(t_plot, c_t_plot)
plt.title(f'Carrier Signal ({fc / 1000} kHz) (First 1%)')
plt.xlabel('Time (ms)')
plt.ylabel('Amplitude')
plt.grid(True)

# FM 调制信号
plt.subplot(6, 1, 3)
plt.plot(t_plot, fm_t_noisy_plot)
plt.title(f'FM Modulated Signal (with AWGN, SNR={snr_dB} dB) (First 1%)')
plt.xlabel('Time (ms)')
plt.ylabel('Amplitude')
plt.grid(True)

# AM 调制信号
plt.subplot(6, 1, 4)
plt.plot(t_plot, am_t_noisy_plot)
plt.title(f'AM Modulated Signal (with AWGN, SNR={snr_dB} dB) (First 1%)')
plt.xlabel('Time (ms)')
plt.ylabel('Amplitude')
plt.grid(True)

# FM 解调信号
plt.subplot(6, 1, 5)
plt.plot(t_plot, fm_demod_signal_plot)
plt.title('FM Demodulated Signal (Filtered, First 1%)')
plt.xlabel('Time (ms)')
plt.ylabel('Amplitude')
plt.grid(True)

# AM 解调信号
plt.subplot(6, 1, 6)
plt.plot(t_plot, am_demod_signal_plot)
plt.title('AM Demodulated Signal (Filtered, First 1%)')
plt.xlabel('Time (ms)')
plt.ylabel('Amplitude')
plt.grid(True)

plt.tight_layout()
plt.savefig('FmAmComparison.svg', format='svg')
plt.close()