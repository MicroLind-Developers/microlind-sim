#pragma once

#include <atomic>
#include <string>

#include <SDL.h>

namespace microlind::gui {

class PcSpeakerAudio {
public:
    PcSpeakerAudio() = default;
    ~PcSpeakerAudio();

    PcSpeakerAudio(const PcSpeakerAudio&) = delete;
    PcSpeakerAudio& operator=(const PcSpeakerAudio&) = delete;

    bool start(std::string& error);
    void shutdown();
    void set_tone(double frequency_hz, float volume, bool enabled);
    [[nodiscard]] bool available() const { return device_ != 0; }

private:
    static void audio_callback(void* userdata, Uint8* stream, int length);
    void render(float* samples, int sample_count);

    SDL_AudioDeviceID device_{};
    int sample_rate_{48000};
    std::atomic<float> frequency_hz_{0.0f};
    std::atomic<float> volume_{0.0f};
    std::atomic_bool enabled_{false};
    double phase_{};
};

} // namespace microlind::gui
