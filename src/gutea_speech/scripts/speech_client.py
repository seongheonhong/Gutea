from google.cloud import speech
from google.cloud import texttospeech as tts
from record_utils import record_to_file
import time, re, os, io, subprocess
from collections import OrderedDict
import rospkg
from gutea_msgs.msg import ClassificationResult

pkg_path = rospkg.RosPack().get_path('gutea_speech')

os.environ["GOOGLE_APPLICATION_CREDENTIALS"] = pkg_path + "/auth/google_authentication.json"

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
    exceptions = {"일본":"1번","이번":"2번"}
    for ex, q in exceptions.items():
        text = re.sub(ex, q, text)

    roomnum = [int(s) for s in re.findall(r'\d+', text)]
    roomnum = list(OrderedDict((r, True) for r in roomnum).keys())

    return roomnum


def play_audio(filename):
    print("Playing audio...")
    subprocess.call(["aplay", filename])


def play_reply(roomnum):
    print(roomnum)
    roomnum_string = ["{}번방".format(num) for num in roomnum]
    room_string = ", ".join(roomnum_string)
    reply = "그럼 " + room_string + "에 가서 사진을 확인한 후 돌아오겠습니다."
    play_tts(reply)

def ask_for_confirmation(roomnum):
    roomnum_string = ["{}번방".format(num) for num in roomnum]
    room_string = ", ".join(roomnum_string)
    reply = room_string + "이 맞습니까?"
    play_tts(reply)

def play_tts(text):
    text_to_speech(text, '/tmp/temp_tts.wav')
    play_audio('/tmp/temp_tts.wav')

def rooms_are_valid(roomnum):
    if len(roomnum) == 0:
        return False
    for num in roomnum:
        if num <= 0 or num > 6:
            return False
    return True

def cb_pre():
    play_audio(pkg_path + '/sounds/coin.wav')


def speak_instructions():
    play_audio(pkg_path + '/sounds/greeting.wav')

    while True:
        play_audio(pkg_path + '/sounds/instruction.wav')
        print("Recording...")
        record_to_file('/tmp/temp.wav', cb_pre)
        try:
            text = recognize_speech('/tmp/temp.wav')
        except Exception as e:
            print("ERROR:", e)
            roomnum = []
        else:
            roomnum = get_directions(text)
        print(roomnum)
        # play_audio('/tmp/temp.wav')

        while not rooms_are_valid(roomnum):
            play_audio(pkg_path + '/sounds/instruction-repeat.wav')
            print("Recording...")
            record_to_file('/tmp/temp.wav', cb_pre)
            try: 
                text = recognize_speech('/tmp/temp.wav')
            except Exception as e:
                print("ERROR:", e)
                roomnum = []
            else:
                roomnum = get_directions(text)
                print(roomnum)
            # play_audio('/tmp/temp.wav')

        ask_for_confirmation(roomnum)
        record_to_file('/tmp/temp2.wav', cb_pre)
        try: 
            text = recognize_speech('/tmp/temp2.wav')
        except Exception as e:
            print("ERROR:", e)
            play_reply(roomnum)
            return roomnum
            break
        for keyword in ["아니", "잘못", "다시", "아닙", "틀렸", "바보", "다른"]:
            if keyword in text:
                print("Retrying...")
                break
        else:
            play_reply(roomnum)
            return roomnum


def speak_results(results):
    label_in_ko = {
        "male_person": "남자",
        "female_person": "여자",
        "dog": "강아지",
        "cat": "고양이",
        "luggage": "캐리어",
        "stroller": "유모차"}
    roomnum_string = ["{}번방에는 {}".format(res.roomnum, label_in_ko[res.label]) for res in results]
    room_string = ", ".join(roomnum_string)
    play_tts(room_string + "가 있습니다.")
    play_audio(pkg_path + '/sounds/closing.wav')


if __name__ == "__main__":
    # speak_instructions()
    print("TEST")
    record_to_file('/tmp/temp.wav', lambda: print("STARTED!"))
    print("RECORDED!")
    text = recognize_speech('/tmp/temp.wav')
    print(text)
