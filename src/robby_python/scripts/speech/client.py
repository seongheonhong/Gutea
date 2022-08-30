from google.cloud import speech
from google.cloud import texttospeech as tts
from speech.record_utils import record_to_file
import time, re, os, io, subprocess
import rospy

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = "/home/sbc-07/catkin_ws/src/robby_python/scripts/speech/google_authentication.json"

def recognize_speech(filename):
    print("Contacting Google for Speech Recognition...")
    client = speech.SpeechClient()
    rec_config = speech.types.RecognitionConfig(
        encoding='LINEAR16',
        language_code='ko-KR',
        sample_rate_hertz=48000
        )
    with io.open(filename,'rb') as audio_file:
        response = client.recognize(
            config=rec_config,
            audio=speech.types.RecognitionAudio(content=audio_file.read())
            )
        for result in response.results:
            for alternative in result.alternatives:
                print('=' * 20)
                print('transcript: ' + alternative.transcript)
                print('confidence: ' + str(alternative.confidence))
    return response.results[0].alternatives[0].transcript


def text_to_speech(text, filename):
    print("Contacting Google for Text-to-Speech...")
    tts_client = tts.TextToSpeechClient()
    synthesis_input = tts.types.SynthesisInput(
        text=text
    )
    voice_params = tts.types.VoiceSelectionParams(
        language_code='ko-KR'
    )
    audio_config = tts.types.AudioConfig(audio_encoding='LINEAR16')
    response = tts_client.synthesize_speech(synthesis_input, voice_params, audio_config)

    with io.open(filename, 'wb') as f:
        f.write(response.audio_content)


def get_directions(text):
    roomnum = [int(s) for s in re.findall(r'\d+', text)]
    return roomnum


def play_audio(filename):
    rospy.loginfo("Playing audio...")
    subprocess.call(["aplay", filename])


def generate_reply(roomnum):
    print(roomnum)
    roomnum_string = ["{}번방".format(num) for num in roomnum]
    room_string = ", ".join(roomnum_string)
    reply = "네, " + room_string + "에 가서 사진을 확인한 후 돌아오겠습니다."
    return reply


def rooms_are_valid(roomnum):
    if len(roomnum) == 0:
        return False
    for num in roomnum:
        if num <= 0 or num > 6:
            return False
    return True

def run_client():
    play_audio('speech/sounds/greeting.wav')
    play_audio('speech/sounds/instruction.wav')
    time.sleep(0.5)
    play_audio('speech/sounds/coin.wav')
    print("Recording...")
    record_to_file('/tmp/temp.wav')
    try:
        text = recognize_speech('/tmp/temp.wav')
    except:
        roomnum = []
    else:
        roomnum = get_directions(text)
        print(roomnum)

    while not rooms_are_valid(roomnum):
        play_audio('speech/sounds/instruction-repeat.wav')
        time.sleep(0.5)
        play_audio('speech/sounds/coin.wav')
        print("Recording...")
        record_to_file('/tmp/temp.wav')
        try: 
            text = recognize_speech('/tmp/temp.wav')
        except:
            roomnum = []
        else:
            roomnum = get_directions(text)
            print(roomnum)

    reply = generate_reply(roomnum)
    print(reply)
    text_to_speech(reply, '/tmp/temp_tts.wav')
    play_audio('/tmp/temp_tts.wav')


