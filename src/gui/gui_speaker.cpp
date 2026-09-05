#include "gui_speaker.hpp"

#include <algorithm>
#include <cmath>

namespace microlind::gui {

PcSpeakerAudio::~PcSpeakerAudio() {
    shutdown();
}

bool PcSpeakerAudio::start(std::string& error) {
    shutdown();

    SDL_AudioSpec desired{};
    desired.freq = 48000;
    desired.format = AUDIO_F32SYS;
    desired.channels = 1;
    desired.samples = 512;
    desired.callback = &PcSpeakerAudio::audio_callback;
    desired.userdata = this;

    SDL_AudioSpec obtained{};
    device_ = SDL_OpenAudioDevice(nullptr, 0, &desired, &obtained, SDL_AUDIO_ALLOW_FREQUENCY_CHANGE);
    if (device_ == 0) {
        error = SDL_GetError();
        return false;
    }
    if (obtained.format != AUDIO_F32SYS || obtained.channels != 1) {
        error = "audio device did not accept mono floating-point samples";
        shutdown();
        return false;
    }

    sample_rate_ = obtained.freq;
    phase_ = 0.0;
    SDL_PauseAudioDevice(device_, 0);
    return true;
}

void PcSpeakerAudio::shutdown() {
    enabled_.store(false, std::memory_order_release);
    if (device_ != 0) {
        SDL_CloseAudioDevice(device_);
        device_ = 0;
    }
}

void PcSpeakerAudio::set_tone(double frequency_hz, float volume, bool enabled) {
    const double maximum = static_cast<double>(sample_rate_) * 0.45;
    const bool audible = enabled && frequency_hz >= 20.0 && frequency_hz <= maximum;
    frequency_hz_.store(static_cast<float>(std::clamp(frequency_hz, 0.0, maximum)), std::memory_order_relaxed);
    volume_.store(std::clamp(volume, 0.0f, 0.25f), std::memory_order_relaxed);
    enabled_.store(audible, std::memory_order_release);
}

void PcSpeakerAudio::audio_callback(void* userdata, Uint8* stream, int length) {
    auto* speaker = static_cast<PcSpeakerAudio*>(userdata);
    speaker->render(reinterpret_cast<float*>(stream), length / static_cast<int>(sizeof(float)));
}

void PcSpeakerAudio::render(float* samples, int sample_count) {
    std::fill_n(samples, sample_count, 0.0f);
    if (!enabled_.load(std::memory_order_acquire)) return;

    const float frequency = frequency_hz_.load(std::memory_order_relaxed);
    const float volume = volume_.load(std::memory_order_relaxed);
    if (frequency <= 0.0f || volume <= 0.0f || sample_rate_ <= 0) return;

    const double phase_step = static_cast<double>(frequency) / static_cast<double>(sample_rate_);
    for (int i = 0; i < sample_count; ++i) {
        samples[i] = phase_ < 0.5 ? volume : -volume;
        phase_ += phase_step;
        phase_ -= std::floor(phase_);
    }
}

} // namespace microlind::gui
