import pyaudio

THRESHOLD = 500
CHUNK_SIZE = 1024
FORMAT = pyaudio.paInt16
BITRATE = 48000

p = pyaudio.PyAudio()
for i in range(p.get_device_count()):
    print(i)
    print(p.get_device_info_by_index(i))
stream = p.open(format=FORMAT, channels=1, input_device_index=6, rate=BITRATE,
    input=True, output=True,
    frames_per_buffer=CHUNK_SIZE)
