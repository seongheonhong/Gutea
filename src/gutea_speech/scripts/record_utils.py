from sys import byteorder
from array import array
from struct import pack

import pyaudio
import wave

# THRESHOLD = 3000
THRESHOLD = 6000
# CHUNK_SIZE = 1024
CHUNK_SIZE = 2048
FORMAT = pyaudio.paInt16
BITRATE = 48000


PAUSE_THRESHOLD = int(1 * BITRATE / CHUNK_SIZE)
print("PAUSE_THRESHOLD:", PAUSE_THRESHOLD)


def is_silent(snd_data):
    "Returns 'True' if below the 'silent' threshold"
    return max(snd_data) < THRESHOLD

def normalize(snd_data):
    "Average the volume out"
    MAXIMUM = 16384
    times = float(MAXIMUM)/max(abs(i) for i in snd_data)

    r = array('h')
    for i in snd_data:
        r.append(int(i*times))
    return r

def trim(snd_data):
    "Trim the blank spots at the start and end"
    def _trim(snd_data):
        snd_started = False
        r = array('h')

        for i in snd_data:
            if not snd_started and abs(i)>THRESHOLD:
                snd_started = True
                r.append(i)

            elif snd_started:
                r.append(i)
        return r

    # Trim to the left
    snd_data = _trim(snd_data)

    # Trim to the right
    snd_data.reverse()
    snd_data = _trim(snd_data)
    snd_data.reverse()
    return snd_data

def add_silence(snd_data, seconds):
    "Add silence to the start and end of 'snd_data' of length 'seconds' (float)"
    r = array('h', [0 for i in range(int(seconds*BITRATE))])
    r.extend(snd_data)
    r.extend([0 for i in range(int(seconds*BITRATE))])
    return r

def record(cb_pre):
    """
    Record a word or words from the microphone and 
    return the data as an array of signed shorts.

    Normalizes the audio, trims silence from the 
    start and end, and pads with 0.5 seconds of 
    blank sound to make sure VLC et al can play 
    it without getting chopped off.
    """
    global record_dev_idx
    # record_dev_idx = 6

    p = pyaudio.PyAudio()
    stream = p.open(format=FORMAT, channels=1, input_device_index=record_dev_idx, rate=BITRATE,
        input=True, output=True,
        frames_per_buffer=CHUNK_SIZE)
    cb_pre()

    num_silent = 0
    snd_started = False

    r = array('h')
    cnt = 0

    while 1:
        cnt+=1
        # little endian, signed short
        snd_data = array('h', stream.read(CHUNK_SIZE, exception_on_overflow=False))
        if byteorder == 'big':
            snd_data.byteswap()
        r.extend(snd_data)

        silent = is_silent(snd_data)
        print(cnt, ",", num_silent, ",", max(snd_data), ",", silent)

        if not snd_started:
            if not silent:
                snd_started = True
        else:
            if silent:
                num_silent += 1
            else:
                if num_silent >= 5:
                    num_silent -= 5

            if num_silent > PAUSE_THRESHOLD:
                break

    sample_width = p.get_sample_size(FORMAT)
    stream.stop_stream()
    stream.close()
    p.terminate()

    r = normalize(r)
    r = trim(r)
    r = add_silence(r, 0.5)
    return sample_width, r

def record_to_file(path, cb_pre):
    "Records from the microphone and outputs the resulting data to 'path'"
    sample_width, data = record(cb_pre)
    data = pack('<' + ('h'*len(data)), *data)

    wf = wave.open(path, 'wb')
    wf.setnchannels(1)
    wf.setsampwidth(sample_width)
    wf.setframerate(BITRATE)
    wf.writeframes(data)
    wf.close()

# Find audio device index
record_dev_idx = 0
p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    dev = p.get_device_info_by_index(i)
    if "USB" in dev.get("name","") and dev.get("maxInputChannels", 0) > 0:
        record_dev_idx = dev["index"]
        print("Found USB recording device at %d!" % record_dev_idx)
        break
else:
    print("ERROR: NO USB RECORDING DEVICE FOUND!")

if __name__ == "__main__":
    record_to_file('/tmp/noise.wav', lambda: print("Started!"))

