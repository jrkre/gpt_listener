#!/bin/python3
import os

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
# from speech_to_text.speech_to_text_msgs import StringArray

from openai import OpenAI


class GPTNode(Node):

    def __init__(self):
        super().__init__('gpt_node')

        #create new gpt instance
        api_key = os.environ.get("OPENAI_API_KEY")
        self.client = OpenAI(api_key=api_key)
        
        
        # pub for tts topic
        self.tts_pub = self.create_publisher(String, '/speech/tts', 10)
                
        
        # takes new object
        self.reset_sub = self.create_subscription(String, '/speech/new_object', self.reset_history, 10)
        # takes user answer and generates a new question
        self.get_new_question = self.create_subscription(String, '/stt/result', self.give_answer, 10)
        # takes the new question in the buffer and speaks it
        self.speak_sub = self.create_subscription(String, '/speech/speak', self.trigger_speech_callback, 10)
        
        self.listen_sub = self.create_subscription(String, '/speech/listen_enable', self.enable_listening, 10)
        
        self.enable = False
        
        self.current_message_history = []
        
        self.speech_buffer = ""
        
        self.file_number = -1
        #self.init_prompt()
    
    def give_answer(self, user_answer):
        self.get_logger().info(f'User answer: "{user_answer.data}"')
        #give answer to gpt's question
        if (self.enable):
            self.speech_buffer = ""

            if (user_answer.data == "stop asking questions"):
                self.speech_buffer = "Okay, what is the next object?"
                return

            self.current_message_history.append({"role": "user", "content": user_answer.data})

            response = self.client.chat.completions.create(
                model="gpt-4o-mini",
                messages=self.current_message_history
            )

            follow_up_question = response.choices[0].message
            # print (follow_up_question.content)

            self.current_message_history.append({"role": "assistant", "content": follow_up_question.content})

            self.speech_buffer = follow_up_question.content
            self.get_logger().info(f'speech buffer: "{self.speech_buffer}"')
            
            self.save_to_file(self.speech_buffer)

            self.enable = False
            msg = String()
            msg.data = self.speech_buffer
        
            self.tts_pub.publish(msg)
        else:
            self.speech_buffer = ""
            pass
        

    def init_prompt(self, object):
        #give prompt engineering init, speak the first question

        initial_prompt = f"""You are a naive robot.  Your task is to ask questions about a given object to 
            learn more about it. The to this chat is input coming from a voice-to-text program and may be unreliable. 
            Start by asking a simple question about the object. The object is {object}"""

        message = {"role": "developer", "content": initial_prompt}
        self.current_message_history.append(message)

        response = self.client.chat.completions.create(
            model="gpt-4o-mini",
            messages=self.current_message_history
        )

        initial_question = response.choices[0].message
        
        self.speech_buffer = initial_question.content

        self.get_logger().info(f'speech buffer: "{self.speech_buffer}"')
        
        #save speech buffer to a new numbered text file 
        self.save_to_file(self.speech_buffer)
        #save speech buffer to a text file
        #with open("speech_buffer.txt", "w") as text_file:
        #    text_file.write(self.speech_buffer)
        
        self.current_message_history.append({"role": "assistant", "content": initial_question.content})
        
        
        msg = String()
        msg.data = self.speech_buffer
        
        self.tts_pub.publish(msg)
        
        self.enable = False
        # print(initial_question.content)
        
        #self.publisher_.publish((data=initial_question))
    
    def reset_history(self, object):
        #reset the history of the conversation
        self.current_message_history = []
        self.init_prompt(object.data)
        
    def trigger_speech_callback(self, msg):
        self.get_logger().info(f'Speaking: "{self.speech_buffer}"')
        
        msg = String()
        msg.data = self.speech_buffer
        
        self.tts_pub.publish(msg)
        
        
    def save_to_file(self, text):
        fp = "results"
        if not os.path.exists(fp):
            os.makedirs(fp)
        if self.file_number == -1:
            self.file_number = len(os.listdir(fp)) + 1
        full_path= os.path.join(fp, f"speech_buffer_{self.file_number}.txt")
        with open(full_path, "w") as text_file:
            text_file.write(self.speech_buffer)
            
        #save file
    
    def enable_listening(self, msg):
        if msg.data == "enable":
            self.enable = True
        else:
            self.enable = False


def main():
    rclpy.init()
    node = GPTNode()
    rclpy.spin(node)


if __name__ == "__main__":
    main()
