#include <chrono>
#include <cstdlib>
#include <iostream>
#include <string>
#include <vector>

#include "NoiseMaker.h"

namespace Synth
{
/////////////////////////////////
// Utilities

double FreqToAngVelocity(const double hz) { return hz * 2.0 * PI; }

struct InstrumentBase;

// a basic node
struct Note
{
    int id;
    double on;
    double off;
    bool isActive;
    InstrumentBase* channel;

    Note() : id(0), on(0.0), off(0.0), isActive(false), channel(nullptr) {}
};

/////////////////////////////////
// Multi-Function Oscillator

enum class Oscillator
{
    Sine,
    Square,
    Triangle,
    SawAnalog,
    SawDigital,
    Noise
};

double osc(const double hz, const double time,
           const Oscillator type = Oscillator::Sine)
{
    switch (type)
    {
    case Oscillator::Sine:
        return std::sin(FreqToAngVelocity(hz) * time);
    case Oscillator::Square:
        return std::sin(FreqToAngVelocity(hz) * time) > 0 ? 1.0 : -1.0;
    case Oscillator::Triangle:
        return std::asin(std::sin(FreqToAngVelocity(hz) * time)) * (2.0 / PI);
    case Oscillator::SawAnalog: {
        double output = 0.0;

        for (double n = 1.0; n < 40.0; ++n)
        {
            output += (std::sin(n * FreqToAngVelocity(hz) * time)) / n;
        }
        return output * (2.0 / PI);
    }
    case Oscillator::SawDigital:
        return (2.0 / PI) * (hz * PI * std::fmod(time, 1.0 / hz) - (PI / 2.0));
    case Oscillator::Noise: // Pseudorandom noise
        return 2.0 * (static_cast<double>(std::rand()) / RAND_MAX) - 1.0;
    default:
        return 0.0;
    }
}

//////////////////////////////////
// Scale to Frequency conversion

constexpr int SCALE_DEFAULT = 0;

double Scale(const int noteId, const int scaleId = SCALE_DEFAULT)
{
    switch (scaleId)
    {
    case SCALE_DEFAULT:
    default:
        // https://stackoverflow.com/questions/1891544/how-can-i-implement-the-piano-key-frequency-function-in-c
        // return 25.96 * std::pow(1.059463094359295, noteId);
        return 8 * std::pow(1.059463094359295, noteId);
    }
}

struct Envelope
{
    virtual double Amplitude(const double time, const double timeOn,
                             const double timeOff) = 0;
};

// Amplitude (Attack, Decay, Sustain, Release) Envelope
class EnvelopeADSR : public Envelope
{
  public:
    explicit EnvelopeADSR(const double attackTime = 0.10,
                          const double decayTime = 0.1,
                          const double startAmplitude = 1.0,
                          const double sustainAmplitude = 1.0,
                          const double releaseTime = 0.2)
        : mAttackTime(attackTime), mDecayTime(decayTime),
          mStartAmplitude(startAmplitude), mSustainAmplitude(sustainAmplitude),
          mReleaseTime(releaseTime)
    {
    }

    double Amplitude(const double time, const double timeOn,
                     const double timeOff) override
    {
        double amplitude = 0.0;

        if (timeOn > timeOff) // Note is on
        {
            const double lifeTime = time - timeOn;

            if (lifeTime <= mAttackTime)
            {
                // In Attack phase - approach max amplitude
                amplitude = (lifeTime / mAttackTime) * mStartAmplitude;
            }
            if (lifeTime > mAttackTime &&
                lifeTime <= (mAttackTime + mDecayTime))
            {
                // In Decay phase - reduce to sustained amplitude
                amplitude = ((lifeTime - mAttackTime) / mDecayTime) *
                                (mSustainAmplitude - mStartAmplitude) +
                            mStartAmplitude;
            }
            if (lifeTime > (mAttackTime + mDecayTime))
            {
                // In Sustain phase - done change till note released
                amplitude = mSustainAmplitude;
            }
        }
        else // Note is off
        {
            const double lifeTime = timeOff - timeOn;
            double releaseAmplitude = 0.0;

            if (lifeTime <= mAttackTime)
            {
                // In Attack phase - approach max amplitude
                releaseAmplitude = (lifeTime / mAttackTime) * mStartAmplitude;
            }
            if (lifeTime > mAttackTime &&
                lifeTime <= (mAttackTime + mDecayTime))
            {
                // In Decay phase - reduce to sustained amplitude
                releaseAmplitude = ((lifeTime - mAttackTime) / mDecayTime) *
                                       (mSustainAmplitude - mStartAmplitude) +
                                   mStartAmplitude;
            }
            if (lifeTime > (mAttackTime + mDecayTime))
            {
                // In Sustain phase - done change till note released
                releaseAmplitude = mSustainAmplitude;
            }

            amplitude =
                ((time - timeOff) / mReleaseTime) * (0.0 - releaseAmplitude) +
                releaseAmplitude;
        }

        // Amplitude should not be negative
        if (amplitude <= 0.0001)
            amplitude = 0.0;

        return amplitude;
    }

  private:
    const double mAttackTime;
    const double mDecayTime;
    const double mStartAmplitude;
    const double mSustainAmplitude;
    const double mReleaseTime;
};

double Env(const double time, Envelope& env, const double timeOn,
           const double timeOff)
{
    return env.Amplitude(time, timeOn, timeOff);
}

struct InstrumentBase
{
    double mVolume;
    Synth::EnvelopeADSR mEnv;
    double mMaxLifeTime;
    std::wstring mName;

    virtual double Sound(const double time, Synth::Note note,
                         bool& isNoteFinished) = 0;

    InstrumentBase(const double attackTime, const double decayTime,
                   const double sustainAmplitude, const double releaseTime,
                   const double maxLifeTime, const std::wstring& name)
        : mVolume(1.0), mEnv(attackTime, decayTime, /*startAmplitude*/ 1.0,
                             sustainAmplitude, releaseTime),
          mMaxLifeTime(maxLifeTime), mName(name)
    {
    }
};

struct InstrumentBell : public InstrumentBase
{
    InstrumentBell()
        : InstrumentBase(/*attackTime*/ 0.01, /*decayTime*/ 1.0,
                         /*sustainAmplitude*/ 0.0, /*releaseTime*/ 1.0,
                         /*maxLifeTime*/ 3.0, /*name*/ L"Bell")
    {
    }

    double Sound(const double time, Synth::Note note,
                 bool& isNoteFinished) override
    {
        double amplitude = Synth::Env(time, mEnv, note.on, note.off);
        if (amplitude <= 0.0)
            isNoteFinished = true;

        double sound =
            +1.00 * Synth::osc(time - note.on, Synth::Scale(note.id + 12),
                               Synth::Oscillator::Sine) +
            0.50 * Synth::osc(time - note.on, Synth::Scale(note.id + 24)) +
            0.25 * Synth::osc(time - note.on, Synth::Scale(note.id + 36));

        return amplitude * sound * mVolume;
    }
};

struct InstrumentBell8 : public InstrumentBase
{
    InstrumentBell8()
        : InstrumentBase(/*attackTime*/ 0.01, /*decayTime*/ 0.5,
                         /*sustainAmplitude*/ 0.8, /*releaseTime*/ 1.0,
                         /*maxLifeTime*/ 3.0, /*name*/ L"8-Bit Bell")
    {
    }

    double Sound(const double time, Synth::Note note,
                 bool& isNoteFinished) override
    {
        double amplitude = Synth::Env(time, mEnv, note.on, note.off);
        if (amplitude <= 0.0)
            isNoteFinished = true;

        double sound =
            +1.00 * Synth::osc(time - note.on, Synth::Scale(note.id),
                               Synth::Oscillator::Square) +
            0.50 * Synth::osc(time - note.on, Synth::Scale(note.id + 12)) +
            0.25 * Synth::osc(time - note.on, Synth::Scale(note.id + 24));

        return amplitude * sound * mVolume;
    }
};

struct InstrumentHarmonica : public InstrumentBase
{
    InstrumentHarmonica()
        : InstrumentBase(/*attackTime*/ 0.05, /*decayTime*/ 1.0,
                         /*sustainAmplitude*/ 0.95, /*releaseTime*/ 0.1,
                         /*maxLifeTime*/ -1.0, /*name*/ L"Harmonica")
    {
    }

    double Sound(const double time, Synth::Note note,
                 bool& isNoteFinished) override
    {
        double amplitude = Synth::Env(time, mEnv, note.on, note.off);
        if (amplitude <= 0.0)
            isNoteFinished = true;

        double sound =
            +1.00 * Synth::osc(time - note.on, Synth::Scale(note.id),
                               Synth::Oscillator::Square) +
            0.50 * Synth::osc(time - note.on, Synth::Scale(note.id + 12),
                              Synth::Oscillator::Square) +
            0.05 * Synth::osc(time - note.on, Synth::Scale(note.id + 24),
                              Synth::Oscillator::Noise);

        return amplitude * sound * mVolume;
    }
};

struct InstrumentDrumKick : public InstrumentBase
{
    InstrumentDrumKick()
        : InstrumentBase(/*attackTime*/ 0.01, /*decayTime*/ 0.15,
                         /*sustainAmplitude*/ 0.0, /*releaseTime*/ 0.0,
                         /*maxLifeTime*/ 1.5, /*name*/ L"Drum Kick")
    {
    }

    double Sound(const double time, Synth::Note note,
                 bool& isNoteFinished) override
    {
        double amplitude = Synth::Env(time, mEnv, note.on, note.off);
        if (amplitude <= 0.0)
            isNoteFinished = true;

        double sound =
            +1.00 * Synth::osc(time - note.on, Synth::Scale(note.id),
                               Synth::Oscillator::Square) +
            0.50 * Synth::osc(time - note.on, Synth::Scale(note.id + 12),
                              Synth::Oscillator::Square) +
            0.05 * Synth::osc(time - note.on, Synth::Scale(note.id + 24),
                              Synth::Oscillator::Noise);

        return amplitude * sound * mVolume;
    }
};

struct InstrumentDrumSnare : public InstrumentBase
{
    InstrumentDrumSnare()
        : InstrumentBase(/*attackTime*/ 0.00, /*decayTime*/ 0.2,
                         /*sustainAmplitude*/ 0.0, /*releaseTime*/ 0.0,
                         /*maxLifeTime*/ 1.0, /*name*/ L"Drum Snare")
    {
    }

    double Sound(const double time, Synth::Note note,
                 bool& isNoteFinished) override
    {
        double amplitude = Synth::Env(time, mEnv, note.on, note.off);
        if (amplitude <= 0.0)
            isNoteFinished = true;

        double sound =
            +1.00 * Synth::osc(time - note.on, Synth::Scale(note.id - 24),
                               Synth::Oscillator::Sine) +
            0.5 * Synth::osc(time - note.on, 0, Synth::Oscillator::Noise);

        return amplitude * sound * mVolume;
    }
};

struct InstrumentDrumHiHat : public InstrumentBase
{
    InstrumentDrumHiHat()
        : InstrumentBase(/*attackTime*/ 0.00, /*decayTime*/ 0.2,
                         /*sustainAmplitude*/ 0.0, /*releaseTime*/ 0.0,
                         /*maxLifeTime*/ 1.0, /*name*/ L"Drum Snare")
    {
    }

    double Sound(const double time, Synth::Note note,
                 bool& isNoteFinished) override
    {
        double amplitude = Synth::Env(time, mEnv, note.on, note.off);
        if (amplitude <= 0.0)
            isNoteFinished = true;

        double sound =
            +1.00 * Synth::osc(time - note.on, Synth::Scale(note.id - 12),
                               Synth::Oscillator::Square) +
            0.9 * Synth::osc(time - note.on, 0, Synth::Oscillator::Noise);

        return amplitude * sound * mVolume;
    }
};

struct Sequencer
{
  public:
    struct Channel
    {
        InstrumentBase* instrument;
        std::wstring beat;
    };

    Sequencer(const float tempo = 120.0f, const int beats = 4,
              const int subBeats = 4)
    {
        mBeats = beats;
        mSubBeats = subBeats;
        mTempo = tempo;
        mBeatTime = (60.f / mTempo) / (float)mSubBeats;
        mCurBeat = 0;
        mTotalBeats = mBeats * mSubBeats;
        mAccumulate = 0;
    }

    size_t Update(const double elapsedTime)
    {
        mVecNotes.clear();
        mAccumulate += elapsedTime;
        while (mAccumulate >= mBeatTime)
        {
            mAccumulate -= mBeatTime;
            mCurBeat++;

            if (mCurBeat >= mTotalBeats)
                mCurBeat = 0;

            int c = 0;
            for (auto v : mVecChannels)
            {
                if (v.beat[mCurBeat] == L'X')
                {
                    Note n;
                    n.channel = mVecChannels[c].instrument;
                    n.isActive = true;
                    n.id = 64;
                    mVecNotes.push_back(n);
                }
                c++;
            }
        }

        return mVecNotes.size();
    }

    void AddInstrument(InstrumentBase* instrument)
    {
        Channel c;
        c.instrument = instrument;
        mVecChannels.push_back(c);
    }

  public:
    int mBeats;
    int mSubBeats;
    double mTempo;
    double mBeatTime;
    double mAccumulate;
    int mCurBeat;
    int mTotalBeats;

    std::vector<Channel> mVecChannels;
    std::vector<Note> mVecNotes;
};
} // namespace Synth

std::vector<Synth::Note> notes;
std::mutex notesMutex;
Synth::InstrumentBell bellInstance;
Synth::InstrumentHarmonica harmonicaInstance;
Synth::InstrumentDrumKick drumKickInstance;
Synth::InstrumentDrumSnare drumSnareInstance;
Synth::InstrumentDrumHiHat drumHiHatInstance;

// Global synthesizer variables
// double frequencyOutput = 0.0;                    // dominant output frequency
// of instrument, i.e. the note EnvelopeADSR envelope; // amplitude modulation
// of output to give texture, i.e. the timbre double octaveBaseFrequency =
// 110.0;              // frequency of octave represensted by keyboard double
// d12thRootOf2 = std::pow(2.0, 1.0 / 12.0); // assuming western 12 notes per
// octave

// Function used by NoiseMaker to generate sound waves
// Return amplitude [-1.0, 1.0] as a function of time
double MakeNoise(const unsigned channel, const double time)
{
    std::scoped_lock<std::mutex> lock(notesMutex);
    double mixedOutput = 0.0;

    // Iterate through all active notes, and mix together
    for (auto& note : notes)
    {
        bool isNoteFinished = false;
        double sound = 0;

        // Get sample for this note by using the correct instrument and envelope
        if (note.channel != nullptr)
        {
            sound = note.channel->Sound(time, note, isNoteFinished);
        }

        // Mix into output
        mixedOutput += sound;

        if (isNoteFinished) // Flag note to be removed
            note.isActive = false;
    }

    notes.erase(std::remove_if(
                    std::begin(notes), std::end(notes),
                    [](Synth::Note const& note) { return !(note.isActive); }),
                std::end(notes));

    return mixedOutput * 0.2; // master volume
}

void PrintDevice(const std::string& device)
{
    std::cout << "Found Output Device: " << device << '\n';
}

void PrintDevice(const std::wstring& device)
{
    std::wcout << "Found Ouput Device: " << device << '\n';
}

void UsingDevice(const std::string& device)
{
    std::cout << "Using device: " << device << '\n';
}

void UsingDevice(const std::wstring& device)
{
    std::wcout << "Using device: " << device << '\n';
}

// TODO:
//  reformat to confirm to cpp17 standard
//  remove all `new` from code
//  breakdown instruments and notes to their separate sections
//  implement string sound
//  implement single producer single consument lock free queue based on folly
//  implement fuzzy test to detect any crashes
//  implement separate class for gui

// Some usefull links
//  https://docs.microsoft.com/en-us/windows/win32/multimedia/devices-and-data-types
//  https://stackoverflow.com/questions/23022349/why-do-parens-prevent-macro-substitution
//  http://sites.music.columbia.edu/cmc/MusicAndComputers/chapter4/04_09.php
//  https://blog.demofox.org/2016/06/16/synthesizing-a-pluked-string-sound-with-the-karplus-strong-algorithm/
//  https://github.com/facebook/folly/blob/master/folly/ProducerConsumerQueue.h
//  https://stackoverflow.com/questions/39680206/understanding-stdhardware-destructive-interference-size-and-stdhardware-cons

int main()
{
    // Get all sound hardware
    const auto devices = NoiseMaker<short>::Enumerate();

    // Display findings
    for (const auto& device : devices)
    {
        PrintDevice(device);
    }
    UsingDevice(devices.front());

    std::cout << '\n'
              << "|   |   |   |   |   | |   |   |   |   | |   | |   |   |   |"
              << '\n'
              << "|   | S |   |   | F | | G |   |   | J | | K | | L |   |   |"
              << '\n'
              << "|   |___|   |   |___| |___|   |   |___| |___| |___|   |   |__"
              << '\n'
              << "|     |     |     |     |     |     |     |     |     |     |"
              << '\n'
              << "|  Z  |  X  |  C  |  V  |  B  |  N  |  M  |  ,  |  .  |  /  |"
              << '\n'
              << "|_____|_____|_____|_____|_____|_____|_____|_____|_____|_____|"
              << '\n'
              << '\n';

    // Create sound machine!!
    NoiseMaker<short> sound(devices.front(), 44100, 1, 8, 256);

    // Link noise function with sound machine
    sound.SetUserFunction(MakeNoise);

    auto clock_old_time = std::chrono::high_resolution_clock::now();
    auto clock_real_time = std::chrono::high_resolution_clock::now();
    double elapsedTime = 0.0;
    double wallTime = 0.0;

    // Establish Sequencer
    Synth::Sequencer seq(90.0);
    seq.AddInstrument(&drumKickInstance);
    seq.AddInstrument(&drumSnareInstance);
    seq.AddInstrument(&drumHiHatInstance);
    seq.AddInstrument(&bellInstance);

    seq.mVecChannels.at(0).beat = L"X...X...X..X.X..";
    seq.mVecChannels.at(1).beat = L"..X...X...X...X.";
    seq.mVecChannels.at(2).beat = L"X.X.X.X.X.X.X.XX";
    seq.mVecChannels.at(3).beat = L"...X...X...X...X";

    // Sit in loop, capturing keyboard state changes and modify
    //  synthesizer output accordingly
    while (1)
    {
        // update-timings
        clock_real_time = std::chrono::high_resolution_clock::now();
        auto time_last_loop = clock_real_time - clock_old_time;
        clock_old_time = clock_real_time;
        elapsedTime = std::chrono::duration<double>(time_last_loop).count();
        wallTime += elapsedTime;
        double timeNow = sound.GetTime();

        // Sequencer
        size_t newNotes = seq.Update(elapsedTime);

        {
            std::scoped_lock<std::mutex> lock(notesMutex);
            for (size_t a = 0; a < newNotes; ++a)
            {
                seq.mVecNotes[a].on = timeNow;
                notes.emplace_back(seq.mVecNotes[a]);
            }
        }

        for (int k = 0; k < 16; ++k)
        {
            const short keyState = GetAsyncKeyState(
                (unsigned char)("ZSXCFVGBNJMK\xbcL\xbe\xbf"[k]));

            // Check if note already exists in currently playing notes
            std::scoped_lock<std::mutex> lock(notesMutex);

            auto noteFound =
                std::find_if(std::begin(notes), std::end(notes),
                             [&k](const Synth::Note& note) {
                                 return note.id == k + 64 &&
                                        note.channel == &harmonicaInstance;
                             });
            if (noteFound == std::end(notes))
            {
                // Note not found in vector
                if (keyState & 0x8000)
                {
                    // key has been pressed so create a new note
                    Synth::Note note;
                    note.id = k + 64;
                    note.on = timeNow;
                    note.channel = &harmonicaInstance;
                    note.isActive = true;

                    notes.emplace_back(note);
                }
            }
            else
            {
                // Note found in vector
                if (keyState & 0x8000)
                {
                    // Key is still held
                    if (noteFound->off > noteFound->on)
                    {
                        // Key has been pressed again during release phase
                        noteFound->on = timeNow;
                        noteFound->isActive = true;
                    }
                }
                else
                {
                    // Key has been released, so switch off
                    if (noteFound->off < noteFound->on)
                    {
                        noteFound->off = timeNow;
                    }
                }
            }
        }

        std::cout << "wall time: " << wallTime << " CPU time: " << timeNow
                  << " Latency: " << wallTime - timeNow << std::endl;
    }

    return 0;
}
