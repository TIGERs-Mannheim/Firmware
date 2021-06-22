#!/usr/bin/python3
"""
This script plays microphone recordings of the bot. If this script is in another directory than the files, change the file name paths accordingly.

Requires the python package pyaudio
"""

import pyaudio

channel0 = open("channel0.rawsound", "rb")
channel1 = open("channel1.rawsound", "rb")

ch0contents = channel0.read()
ch1contents = channel1.read()

def chunks(bts):
    for i in range(0, len(bts), 2):
        yield bts[i:i + 2]

stereo = [e[0] + e[1] for e in zip(list(chunks(ch0contents)), list(chunks(ch1contents)))]
r = b''
for e in stereo:
    r += e

p = pyaudio.PyAudio();

stream = p.open(format=pyaudio.paInt16, channels=2, rate=16000, output=True)

stream.write(r)

stream.stop_stream()
stream.close()
p.terminate()
