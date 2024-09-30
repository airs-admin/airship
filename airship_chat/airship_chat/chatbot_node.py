import json
import os
import subprocess
import threading
import time
import wave

import numpy as np
import pyaudio
import rclpy
import yaml
from openai import OpenAI
from pocketsphinx import LiveSpeech, get_model_path
from rclpy.node import Node

from airship_interface.srv import AirshipInstruct

class voice_assistant(Node):
    """
    Voice assistant node that listens for commands and processes them.
    """
    def __init__(self):
        super().__init__("chatbot_node")
        self.get_logger().info(f"chatbot_node service start!")

        self.declare_parameter('config', 'config/config.yaml')
        self.config_path = self.get_parameter('config').get_parameter_value().string_value
        with open(self.config_path, 'r') as file:
            self.config = yaml.safe_load(file)
        self.speech_record_path = self.config.get('speech_record_path')
        self.wakeup_audio_path = self.config.get('wakeup_audio_path')
        self.player_device_index = self.config.get('player_device_index')
        self.sphinx_model_path = self.config.get('sphinx_model_path')
        self.sphinx_model_name = self.config.get('sphinx_model_name')
        self.openai_api_key = self.config.get('openai_api_key')
        self.openai_base_url = self.config.get('openai_base_url')
        self.wakeup_audio_finish = False

        self.text_processor_client = self.create_client(AirshipInstruct, '/airship_planner/planner_server')
        if not self.text_processor_client.wait_for_service(timeout_sec=10.0):
            self.llm_server_ready = False
            return
        self.req = AirshipInstruct.Request()
        self.recognition_thread = threading.Thread(target=self.recognize_speech)
        self.recognition_thread.start()

    def play_audio(self, file_path):
        """
        Plays an audio file.
        
        Args:
            file_path (str): Path to the audio file.
        """
        player_device = "plug:SLAVE='" + self.player_device_index + "'"

        try:
            subprocess.run(['aplay', '-D', player_device, file_path], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            self.wakeup_audio_finish = True
        except subprocess.CalledProcessError as e:
             self.get_logger().error(f"Failed to play audio: {file_path}. Error: {e}")

    def record_auto(self):
        """
        Records audio and saves it to a file.
        """
        CHUNK = 1024
        FORMAT = pyaudio.paInt16
        CHANNELS = 1
        RATE = 16000
        MIN_DB = 2000
        DELAY_TIME_SECONDS = 1

        audio = pyaudio.PyAudio()
        stream = audio.open(format=FORMAT,
                            channels=CHANNELS,
                            rate=RATE,
                            input=True,
                            frames_per_buffer=CHUNK)
        frames = []
        is_recording = False
        continue_recording = True
        is_quiet = False
        quiet_start_time = 0

        self.get_logger().info('Say something!')
        try:
            while continue_recording:
                data = stream.read(CHUNK, exception_on_overflow=False)
                frames.append(data)
                audio_data = np.frombuffer(data, dtype=np.short)
                current_max = np.max(audio_data)

                if current_max > MIN_DB and not is_recording:
                    is_recording = True
                    self.get_logger().info("Start recording...")
                    quiet_start_time = 0

                if is_recording:
                    if current_max < MIN_DB and not is_quiet:
                        is_quiet = True
                        quiet_start_time = 0

                    if current_max > MIN_DB:
                        is_quiet = False
                        quiet_start_time = 0

                    if quiet_start_time > DELAY_TIME_SECONDS * RATE / CHUNK and is_quiet:
                        continue_recording = False
                        self.get_logger().info("End of recording due to silence.")
                    else:
                        quiet_start_time += 1

        except Exception as e:
            self.get_logger().error(f"Recording failed: {e}")
        finally:
            stream.stop_stream()
            stream.close()
            audio.terminate()

            with wave.open(self.speech_record_path, 'wb') as wf:
                wf.setnchannels(CHANNELS)
                wf.setsampwidth(audio.get_sample_size(FORMAT))
                wf.setframerate(RATE)
                wf.writeframes(b''.join(frames))
            self.get_logger().info("Stop recording!")

    def whisper_STT(self, audio_file_path):
        """
        Transcribes audio using Whisper API.
        
        Args:
            audio_file_path (str): Path to the audio file.
        
        Returns:
            str: Transcribed text.
        """
        try:
            client = OpenAI(api_key=self.openai_api_key, base_url=self.openai_base_url)
            audio_file = open(audio_file_path, "rb")
            transcript = client.audio.transcriptions.create(
                            model="whisper-1",
                            file=audio_file, 
                            response_format="json"
                        )
        except Exception as e:
            self.get_logger().error(f"Transcription failed: {e}")

        return transcript.text

    def Sphinx_initialization(self):
        """
        Sets up PocketSphinx for keyword detection.
        
        Returns:
            LiveSpeech: PocketSphinx LiveSpeech object.
        """
        try:
            speech = LiveSpeech(
                verbose=False,
                sampling_rate=16000,
                buffer_size=2048,
                no_search=False,
                full_utt=False,
                hmm=os.path.join(get_model_path(),'en-us/en-us'),
                lm =os.path.join(self.sphinx_model_path,  self.sphinx_model_name +'.lm'),
                dic=os.path.join(self.sphinx_model_path,  self.sphinx_model_name +'.dic')
            )
        except Exception as e:
            self.get_logger().error(f"Sphinx initialization failed: {e}")

        return speech

    def process_response(self, future):
        """
        Processes the response from the LLM server.
        
        Args:
            future (Future): Future object containing the response.
        """
        try:
            result = future.result()
            ret_value = result.ret
            response_output = {
                0: "Success!",
                1: "The robot is working!",
                2: "The instruction is empty!",
                3: "Navigation error!",
                4: "Did not find the item!",
                5: "Failed to grab the item!",
                6: "Failed to place the item!"
            }
            response = response_output.get(ret_value, "Unknown response")

            self.get_logger().info(f'Received response form LLM server: {response}')
            self.llm_server_ready = True
        except Exception as e:
            self.get_logger().error(f'LLM server call failed: {e}')

    def recognize_speech(self):
        """
        Recognizes speech and processes commands.
        """
        try:
            sphinx_speech = self.Sphinx_initialization()
            self.get_logger().info("start listening...")

            for phrase in sphinx_speech:
                if not self.text_processor_client.wait_for_service(timeout_sec=1.0):
                    self.get_logger().info(f'Trying to connect to the LLM server...')
                    self.llm_server_ready = False
                    for i in range(5):
                        if self.text_processor_client.wait_for_service(timeout_sec=1.0):
                            self.llm_server_ready = True
                            break
                        else:
                            self.get_logger().info(f'Trying to connect to the LLM server, attempt {i+1}/5')

                if self.llm_server_ready:
                    if str(phrase) in ["airship", "air ship", "AIR SHIP", "AIRSHIP"]:
                        self.get_logger().info(f"Keyword detected: {phrase}！")
                        self.play_audio(self.wakeup_audio_path)
                        while not self.wakeup_audio_finish:
                            time.sleep(0.1)
                        self.record_auto()
                        start = time.time()
                        text = self.whisper_STT(self.speech_record_path)
                        self.get_logger().info(f"Time for STT: {round(time.time() - start, 2)} s.")
                        self.get_logger().info(f'STT result：{text}')

                        self.req.msg = text
                        self.future = self.text_processor_client.call_async(self.req)
                        self.future.add_done_callback(self.process_response)
                        break
                else:
                    self.get_logger().info('LLM service not available, please check it!')
                    break
        except Exception as e:
            self.get_logger().error(f"Chat failed: {e}")
            
def main(args=None):
    rclpy.init(args=args)
    chatbot = voice_assistant()

    if not chatbot.llm_server_ready:
        chatbot.get_logger().info('LLM service not available, please check it!')
    else:
        try:
            rclpy.spin(chatbot)
        except KeyboardInterrupt:
            pass

    chatbot.get_logger().info("Shutting down!")
    chatbot.destroy_node()
    rclpy.shutdown()
        

if __name__ == '__main__':
    main()