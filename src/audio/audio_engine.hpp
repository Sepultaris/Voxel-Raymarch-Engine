#pragma once

struct ALCcontext;
struct ALCdevice;

namespace voxel {

class AudioEngine final {
public:
    AudioEngine();
    ~AudioEngine();

    AudioEngine(const AudioEngine&) = delete;
    AudioEngine& operator=(const AudioEngine&) = delete;

    [[nodiscard]] bool available() const noexcept { return context_ != nullptr; }

private:
    ALCdevice* device_{};
    ALCcontext* context_{};
};

} // namespace voxel

