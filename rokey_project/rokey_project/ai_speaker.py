import os
import time
import openai
import sounddevice as sd
import scipy.io.wavfile as wav
import numpy as np
from scipy.signal import resample
from openwakeword.model import Model
from ament_index_python.packages import get_package_share_directory
from rokey_interfaces.msg import Medicine, MedicineArray
from rokey_interfaces.msg import TaskState
from rokey_interfaces.msg import RobotState
from rclpy.node import Node
import rclpy
import json
import asyncio
import edge_tts
import re


class MedicinePublisher(Node):
    def __init__(self):
        super().__init__('Medicine_publisher')
        self.publisher = self.create_publisher(MedicineArray, '/medicine', 10)

    def publish_bulk(self, meds): 
        msg = MedicineArray()
        for item in meds:
            m = Medicine()
            m.name = item["name"]
            m.count = int(item["count"])
            msg.medicines.append(m)
        self.publisher.publish(msg)
        self.get_logger().info(f" Published: {[f'{m.name} {m.count}개' for m in msg.medicines]}")

class FeedbackNode(Node):
    def __init__(self):
        super().__init__('feedback_node')
        self.sub = self.create_subscription(TaskState, '/task_state', self.task_callback, 10)
        self.get_logger().info("feedback_node (약 설명 음성 출력) 시작됨")

    def task_callback(self, msg):
        if msg.state == "explain_medicine":
            medicine_names = [name.strip() for name in msg.qr_info.split(" ")]
            self.get_logger().info(f"약 설명 요청 수신: {medicine_names}")
            explanation = explain_medicines_from_list(medicine_names)
            self.get_logger().info(f"설명 내용: {explanation}")
            speak("음음 각 약의 설명입니다")
            speak(explanation)
            speak("음음 감사합니다 안녕히 가세요")

        elif msg.state == "empty":
            medicine_names = [name.strip() for name in msg.qr_info.split(",") if name.strip()]

            if not medicine_names:
                return

            if len(medicine_names) == 1:
                name = medicine_names[0]
                josa = choose_josa(name, ("이", "가"))
                message = f"{name}{josa} 비어있습니다. 채워주세요."
            else:
                joined_names = " / ".join(medicine_names)
                # 복수일 경우엔 마지막 단어에 대해 은/는 조사 적용
                last_name = medicine_names[-1]
                josa = choose_josa(last_name, ("은", "는"))
                message = f"{joined_names}{josa} 모두 비어있습니다. 채워주세요."

            self.get_logger().info(f"약 없음 알림: {message}")
            speak(message)


class QRPromptNode(Node):
    def __init__(self):
        super().__init__('qr_prompt_node')
        self.sub = self.create_subscription(TaskState, '/task_state', self.callback, 10)
        self.last_prompted = False

    def callback(self, msg):
        if msg.state == 'detected':
            if not self.last_prompted:
                speak("음음 안녕하세요 로키 약국입니다 QR을 스캔하거나 hello rokey를 말해주세요")
                self.last_prompted = True
        else:
            self.last_prompted = False
            


rclpy.init()
publisher = MedicinePublisher()
subscriber = FeedbackNode()
qr_node = QRPromptNode()


# OpenAI API 키 설정
openai.api_key = os.getenv("OPENAI_API_KEY")
if openai.api_key is None:
    raise ValueError("OPENAI_API_KEY 환경 변수가 설정되어 있지 않습니다.")

package_share = get_package_share_directory("rokey_project2")
MODEL_PATH = os.path.join(package_share, "models", "hello_rokey_8332_32.tflite")

# 웨이크워드 설정
MODEL_NAME = "hello_rokey_8332_32.tflite"
model = Model(wakeword_models=[MODEL_PATH])
wakeword_name = os.path.basename(MODEL_PATH).split(".")[0]
listening = True
last_trigger_time = 0
TRIGGER_COOLDOWN = 5  # 웨이크워드 재감지 방지 시간 (초)
last_confidence_reset_time = 0
CONFIDENCE_RESET_COOLDOWN = 3  # 초 단위


# 마이크 설정
BUFFER_SIZE = 1600  # 약 0.1초 분량 (16000 Hz 기준)
SAMPLING_RATE = 16000
stream = sd.InputStream(samplerate=SAMPLING_RATE, channels=1, dtype='int16')
stream.start()

# 현재 재고
VALID_OVER_THE_COUNTER = ["타이레놀", "코대원", "붕대", "파스"]

# 한글과 영문 mapping
KOR_TO_ENG = {
    "붕대": "bandage",
    "타이레놀": "tylenol",
    "파스": "sore_patch",
    "코대원": "codaewon_syrup"
}

# 은/는, 이/가 구분 출력
def choose_josa(word, josa_pair):
    """
    단어의 마지막 글자에 받침이 있으면 josa_pair[0], 없으면 josa_pair[1]
    예) ("이", "가") 또는 ("은", "는")
    """
    if not word:
        return josa_pair[1]  # 기본값
    last_char = word[-1]
    code = ord(last_char) - 0xAC00
    has_final = (code % 28) != 0
    return josa_pair[0] if has_final else josa_pair[1]


#음성 수신 함수
def record_audio(filename="input.wav", duration=4, fs=16000):
    print("🎤 사용자 음성 수신 중...")
    audio = sd.rec(int(duration * fs), samplerate=fs, channels=1, dtype='int16')
    sd.wait()
    wav.write(filename, fs, audio)
    print("녹음 완료")

# whisper-1 모델 입력 함수
def transcribe_audio(filename="input.wav"):
    try:
        with open(filename, "rb") as f:
            transcript = openai.Audio.transcribe("whisper-1", f, language="ko")
        text = transcript.get("text", "").strip()
        return text
    except Exception as e:
        print(f"[Whisper 오류] {e}")
        return ""

# def transcribe_audio(filename="input.wav"):
#     with open(filename, "rb") as f:
#         transcript = openai.Audio.transcribe("whisper-1", f, language="ko")
#     return transcript["text"]
# def transcribe_audio(filename="input.wav"):
#     with open(filename, "rb") as f:
#         transcript = openai.Audio.transcribe("whisper-1", f, language="ko")
#     # 공백 제거하여 분석 정확도 향상
#     return transcript["text"].replace(" ", "")


#모델 초기화 함수
def reset_model():
    global model
    model = Model(wakeword_models=[MODEL_NAME])

# 버퍼 초기화 함수
def flush_wakeword_buffer(duration=1.0, fs=16000):
    # 무음 데이터를 생성하여 모델에 입력
    silence = np.zeros(int(fs * duration), dtype=np.int16)
    resampled = resample(silence, int(len(silence) * 16000 / fs))
    model.predict(resampled)
    print("[버퍼 초기화] wakeword 모델 버퍼를 무음으로 덮어썼습니다.")

# 전문의약품 / 비처방의약품 판단 함수
def analyze_medicine_command(text):
    prompt = (
        f'다음은 사용자 음성 명령입니다: "{text}"\n'
        "너는 한국에서 아주 유능한 약사야.\n"
        "이 명령에 포함된 의약품의 이름과 수량을 JSON 배열 형식으로 정확하게 추출해줘. "
        "그리고 약의 데이터베이스를 확인하고, 해당 약이 전문의약품인지, 처방전 없이 살 수 있는 비처방약인지 판단해줘.\n"
        "절대 거짓말을 해선 안 되며, 모르면 모른다고 대답해줘. 절대 친절하게 설명하지 마.\n\n"
        "그리고 붕대나 파스같은 것도 추출해야해."
        "사용자가 말한 단어가 음성 인식 오류일 가능성이 있으므로, 한국 의약품 이름 기준으로 유사한 이름으로 보정해줘."
        "예를 들어: 바카스 → 박카스, 박파스 → 박카스, 타리에놀 → 타이레놀, 고데온 → 코대원, 패스 → 파스, 군대 → 붕대, "
        "**중요:** 반드시 아래와 같이 **마크다운 코드 블록 없이** []로 시작하는 **JSON 본문만** 출력해야 해. 예시는 다음과 같아:\n\n"
        '[\n'
        '  {\n'
        '    "valid": [\n'
        '      {"name": "타이레놀", "count": 1}\n'
        '    ],\n'
        '    "restricted": [\n'
        '      {"name": "졸피뎀", "count": 1}\n'
        '    ]\n'
        '  }\n'
        ']\n\n'
        "설명은 절대 하지 말고, JSON 본문만 응답해. 전문의약품은 restricted에 넣고, 나머지는 valid에 넣어."
    )

    response = openai.ChatCompletion.create(
        model="gpt-4o",
        messages=[{"role": "user", "content": prompt}]
    )
    result = response.choices[0].message["content"]

    # JSON 파싱 후 valid 항목 필터링
    try:
        parsed = json.loads(result)
        if isinstance(parsed, list) and parsed:
            parsed_obj = parsed[0]
            parsed_obj["valid"] = [
                med for med in parsed_obj.get("valid", [])
                if med["name"] in VALID_OVER_THE_COUNTER
            ]
            return json.dumps([parsed_obj], ensure_ascii=False)
    except Exception as e:
        print(f"[JSON 필터링 실패] {e}")

    return result  # 파싱 실패 시 원본 반환

# 약 추천 함수
def recommend_medicine_for_symptom(symptom_text):
    prompt = (
        f"""사용자가 이렇게 말했습니다: \"{symptom_text}\"\n"
        "이건 약 이름이나 수량이 아니라 증상일 가능성이 높아.\n"
        "너는 한국의 전문 약사야. 이 증상에 적절한 일반의약품을 추천해줘.\n"
        "약 이름과 간단한 효능을 알려주고, 전문의약품이 필요하거나 병원 방문이 필요한 경우엔 그렇게 말해줘.\n"
        "절대 증상별로 따로따로 약을 나열하지 마. 전체 문장을 이해하고 통합된 처방처럼 응답해.\n\n"
        특히 '근육통', '허리통증', '손목 통증' 같은 외부 통증에는 파스, 진통제 등을 우선 추천해\n.
        "졸림, 피로, 집중력 저하 등에는 박카스, 비타500, 오로나민C 같은 자양강장제도 추천 가능해.\n"
        "절대 증상별로 따로따로 약을 나열하지 마. 전체 문장을 이해하고 통합된 처방처럼 응답해.\n\n"
        "반드시 다음 출력 예시를 따라서 대답해"
        "출력 예시:\n"
        "- 추천약은 타이레놀 (해열진통제)입니다.\n"
        "- 증상이 지속되면 병원에 방문하세요.\n"
        "- 추천약은 파스입니다.\n"
        "- 증상이 지속되면 병원에 방문하세요.\n"
        "- 추천약은 타이레놀 (해열진통제)과 판콜에이 (감기약)입니다.\n"
        "- 증상이 계속되면 병원에 방문하세요."""
    )

    response = openai.ChatCompletion.create(
        model="gpt-4o",
        messages=[{"role": "user", "content": prompt}]
    )
    return response.choices[0].message["content"]

# 추천약 줄지 안줄지 물어보는 함수
def extract_medicines_from_response(response_text):
    prompt = (
        f"다음은 사용자에게 안내한 추천 약 설명입니다:\n\"{response_text}\"\n"
        "여기에서 실제 추천된 일반의약품 이름만 JSON 배열로 출력해줘.\n"
        "사용자가 말한 단어가 음성 인식 오류일 가능성이 있으므로, 한국 의약품 이름 기준으로 유사한 이름으로 보정해줘.\n"
        "예를 들어: 바카스 → 박카스, 박파스 → 박카스, 타리에놀 → 타이레놀\n"
        "설명, 마크다운, 코드블록 없이 반드시 JSON 배열만 출력해. 예: [\"타이레놀\", \"파스\"]\n"
        "만약 추천된 약이 없다면 빈 배열 []을 출력해."
    )

    try:
        response = openai.ChatCompletion.create(
            model="gpt-4o",
            messages=[{"role": "user", "content": prompt}]
        )
        content = response.choices[0].message["content"].strip()


        return json.loads(content)
    except Exception as e:
        print(f"[추천 약 추출 실패] {e}")
        return []



# def parse_partial_medicine_request(user_text, recommended_names):
#     prompt = (
#         f"사용자가 이렇게 말했습니다: \"{user_text}\"\n"
#         f"이전에 추천된 약 목록은 다음과 같습니다: {', '.join(recommended_names)}\n"
#         "사용자의 말에서 요청한 약의 이름만 리스트로 출력해줘. "
#         "추천된 약 중 일부만 요청하거나, 특정 약을 빼달라는 말일 수도 있어.\n"
#         "예:\n"
#         "- 입력: '타이레놀만 주세요' → 출력: [\"타이레놀\"]\n"
#         "- 입력: '케토톱은 빼고 주세요' → 출력: [\"타이레놀\"]\n"
#         "- 입력: '그 약 말고 다른 거 주세요' → 출력: []\n\n"
#         "JSON 배열 형식으로 출력해. 예: [\"타이레놀\"]"
#     )

#     response = openai.ChatCompletion.create(
#         model="gpt-4o",
#         messages=[{"role": "user", "content": prompt}]
#     )

#     try:
#         parsed = json.loads(response.choices[0].message["content"])
#         return parsed
#     except Exception:
#         return []

def normalize_korean(text):
    return re.sub(r"\s+", "", text).lower()

def parse_partial_medicine_request(answer_text, options):
    normalized_answer = normalize_korean(answer_text)
    matched = []
    for name in options:
        if normalize_korean(name) in normalized_answer:
            matched.append(name)
    return matched

# 말하는 함수
def speak(text):
    try:
        loop = asyncio.get_running_loop()
    except RuntimeError:
        loop = None

    if loop and loop.is_running():
        asyncio.run_coroutine_threadsafe(edge_speak(text), loop)
    else:
        asyncio.run(edge_speak(text))

# edge tts용 speak함수
async def edge_speak(text):
    communicate = edge_tts.Communicate(
        text=text,
        voice="ko-KR-SunHiNeural",
        rate="+20%"
    )
    await communicate.save("response.mp3")
    os.system("mpg123 response.mp3")

def get_user_response():
    record_audio(filename="yesno.wav", duration=3)
    return transcribe_audio(filename="yesno.wav")

def is_invalid_whisper_result(text):
    return not text or text.strip() in [
        "시청해 주셔서 감사합니다.",
        "감사합니다.",
        "안녕하세요.",
        "네.",
        "예."
    ]

# ai_speaker 로직
def run_ai_speaker():
    record_audio()
    text = transcribe_audio()

    if is_invalid_whisper_result(text):
        speak("음음 음성이 잘 들리지 않았어요 다시 말씀해 주세요")
        return
    
    print(f"\n[음성 인식 결과] {text}\n")

    response = analyze_medicine_command(text)
    print(f"[GPT 응답] {response}\n")

    try:
        parsed = json.loads(response)
        if isinstance(parsed, list):
            if not parsed:
                raise ValueError("의약품 분석 결과가 없습니다.")
            parsed = parsed[0]

        valid = parsed.get("valid", [])
        restricted = parsed.get("restricted", [])

        # 유효한 약이 있고, 사용자 명령에 직접 약 이름이 포함되어 있으면 바로 준비
        out_of_stock = [m for m in valid if m["name"] not in VALID_OVER_THE_COUNTER]
        in_stock = [m for m in valid if m["name"] in VALID_OVER_THE_COUNTER]

        # 재고 없는 약 알림
        if out_of_stock:
            speak("음음"+", ".join([m["name"] for m in out_of_stock]) + "은 현재 재고가 없습니다.")

        # 재고 있는 약 처리
        if in_stock:
            speak("음음"+", ".join([f'{m["name"]} {m["count"]}개' for m in in_stock]) + "을 준비하겠습니다.")
            publisher.publish_bulk([
                {
                    "name": KOR_TO_ENG.get(item["name"], item["name"]),
                    "count": item["count"]
                } for item in in_stock
            ])
            return


        # 추천은 됐지만, 사용자 응답이 필요할 경우
        # if valid:
        #     valid_names = [m["name"] for m in valid]
        #     speak("추천 약은 " + "와 ".join(valid_names) + "입니다. 어떤 약을 드릴까요?")
        #     selected = []
        #     retry_count = 0
        #     MAX_RETRIES = 2

        #     while not selected and retry_count < MAX_RETRIES:
        #         answer = get_user_response()
        #         selected = get_selected_medicines(answer, recommended_names)

        #         if selected:
        #             break
        #         else:
        #             retry_count += 1
        #             if retry_count < MAX_RETRIES:
        #                 speak("죄송해요, 다시 한 번 어떤 약을 드릴지 말씀해 주세요.")

        #     if selected:
        #         print(f"[선택된 약 목록] {selected}")
        #         publisher.publish_bulk([
        #                 {
        #                     "name": KOR_TO_ENG.get(name, name),
        #                     "count": 1
        #                 } for name in selected
        #             ])
        #         speak(", ".join(selected) + "을 준비하겠습니다.")
        #     else:
        #         speak("선택된 약이 없습니다.")

        #     return

        if restricted:
            restricted_names = [m["name"] for m in restricted]
            speak("음음"+" / ".join(restricted_names) + "은 약사와 상담이 필요합니다.")
            return

        # 의약품 분석 실패 시 → 증상 기반 추천
        raise ValueError("의약품이 파악되지 않음")

    except Exception as e:
        print(f"[약 분석 실패 → 증상으로 간주] {e}")
        final_response = recommend_medicine_for_symptom(text)
        print(f"[추천 결과] {final_response}")
        speak(f"음음 {final_response}")

        # 추천 약들 이름 추출
        recommended_names = extract_medicines_from_response(final_response)
        print(f"[추천 약 추출 응답] {recommended_names}")

        # 재고 있는 약만 필터링
        in_stock = [name for name in recommended_names if name in VALID_OVER_THE_COUNTER]
        out_of_stock = [name for name in recommended_names if name not in VALID_OVER_THE_COUNTER]

        if out_of_stock:
            speak("음음"+", ".join(out_of_stock) + "은 현재 재고가 없습니다.")

        if in_stock:
            speak("음음 추천된 약은" + "와 ".join(in_stock) + "입니다 어떤 약을 드릴까요")
            selected = []
            retry_count = 0
            MAX_RETRIES = 3

            while not selected and retry_count < MAX_RETRIES:
                answer = get_user_response()
                selected = get_selected_medicines(answer, in_stock)

                if selected:
                    break
                else:
                    retry_count += 1
                    if retry_count < MAX_RETRIES:
                        speak("음음 죄송해요 다시 한 번 어떤 약을 드릴지 말씀해 주세요")

            print(f"[선택된 약 목록] {selected}")
            if selected:
                publisher.publish_bulk([
                        {
                            "name": KOR_TO_ENG.get(name, name),
                            "count": 1
                        } for name in selected
                ])
                speak(", ".join(selected) + "을 준비하겠습니다.")
            else:
                speak("음음 선택된 약이 없습니다.")

        else:
            speak("음음 해당 증상에 대한 약은 현재 재고가 없습니다.")


# wakeupword 탐지 함수
def detect_wakeword():
    global last_confidence_reset_time
    audio_chunk, _ = stream.read(BUFFER_SIZE)
    audio_chunk = np.squeeze(audio_chunk)
    audio_chunk = resample(audio_chunk, int(len(audio_chunk) * 16000 / SAMPLING_RATE))
    outputs = model.predict(audio_chunk, threshold=0.1)
    confidence = outputs[wakeword_name]

    # 일정 시간 이내에는 confidence 무시
    if time.time() - last_confidence_reset_time < CONFIDENCE_RESET_COOLDOWN:
        print("[쿨다운 중] confidence 무시")
        return False

    print(f"[웨이크워드 감지] confidence={confidence:.2f}")
    if confidence > 0.6:
        last_confidence_reset_time = time.time()
        return True
    return False

def get_selected_medicines(answer_text, recommended_names):
    norm_answer = normalize_korean(answer_text)
    selected = []
    for name in recommended_names:
        if normalize_korean(name) in norm_answer:
            selected.append(name)
    return selected

#약에 대한 설명하는 함수
def explain_medicines_from_list(medicine_names):
    joined = ", ".join(medicine_names)
    prompt = (
        f"너는 한국의 전문 약사야. 사용자가 다음 약을 건네받았습니다: {joined}.\n"
        "각 약에 대해 아주 간단하게 1문장 정도로 효능과 주의사항을 요약해서 말해줘.\n"
        "절대 설명이 길면 안 돼. 각각의 약 이름과 그 설명만 아주 간단하게 나열해. 마크다운, 번호 없이.\n"
        "예를 들어:\n"
        "타이레놀: 해열진통제이며, 과다 복용 시 간 손상 위험이 있습니다.\n"
        "박카스: 피로회복제로 사용되며, 카페인이 포함되어 있습니다.\n"
    )

    response = openai.ChatCompletion.create(
        model="gpt-4o",
        messages=[{"role": "user", "content": prompt}]
    )
    return response.choices[0].message["content"]

# main
def main():
    print("'hello rokey' 웨이크워드 감지 대기 중...")

    def ros_spin_thread():
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(subscriber)
        executor.add_node(qr_node)
        executor.spin()

    import threading
    ros_thread = threading.Thread(target=ros_spin_thread)
    ros_thread.daemon = True
    ros_thread.start()


    global last_trigger_time, listening
    last_trigger_time = 0
    listening = True

    while True:
        if listening and detect_wakeword() and time.time() - last_trigger_time > TRIGGER_COOLDOWN:
            listening = False
            last_trigger_time = time.time()

            # 웨이크워드 감지 완전 중지
            global model_enabled
            model_enabled = False

            stream.stop()
            print("웨이크워드 감지됨! 음성 명령 수신 시작.")
            speak("음음 무엇을 도와드릴까요 증상이나 원하시는 의약품을 말씀해주세요.")
            run_ai_speaker()
            flush_wakeword_buffer(duration=1)
            print("명령 처리 완료, 다시 웨이크워드 대기 중...")

            time.sleep(TRIGGER_COOLDOWN)

            stream.start()
            model_enabled = True  # 웨이크워드 감지 재개
            listening = True

        time.sleep(0.1)


# 아래는 삭제하거나 유지해도 무방함
if __name__ == "__main__":
    main()