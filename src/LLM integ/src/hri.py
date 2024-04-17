from openai import OpenAI
import rospy
from std_msgs.msg import String

rospy.init_node('LLM_HVI', anonymous=True)
pub = rospy.Publisher('/LLM_HVI', String, queue_size=10)

client = OpenAI(
    api_key="Enter your api-key",
)


def interactive_chat(last_output):
    # keyboard input
    human_input = input("Human command: ")

    messages = [
                {
                    "role": "system",
                    "content": "You are an intelligent autonomous vehicle helper tasked with selecting driving policies based on human instructions. You will receive input in the form of a JSON object specifying the current driving policy settings. Your job is to adjust these settings within the specified ranges and output the modified policy as a JSON object. The parameters to be adjusted are 'top_speed_straight', 'top_speed_corner', and 'acceleration'. Ensure that 'top_speed_straight' is between 8 and 15 (0 to stop, park, pedestrian etc.), 'top_speed_corner' is less than 'top_speed_straight', and 'acceleration' is between 1 and 5. Your output should be a valid JSON object reflecting the updated driving policy. For example, given the input {\"top_speed_straight\": 10, \"top_speed_corner\": 5, \"acceleration\": 1}, you should output an adjusted policy within the allowed ranges."
                },
                {"role": "assistant", "content": f'Pervious settings: {last_output}'},
                {"role": "user", "content": human_input},
            ]

    prediction = client.chat.completions.create(
            model="gpt-3.5-turbo",
            messages=messages,
            temperature=0.9,
            max_tokens=500,
            top_p=1,
            frequency_penalty=0,
            presence_penalty=0.6,
            # stop=[""]
            )
    
    reply = prediction.choices[0].message.content
    print(reply)
    pub.publish(String(reply))
    return reply

last_output = "None"
while(1):
    try:
        last_output = interactive_chat(last_output)
    except rospy.ROSInterruptException:
        exit()

    # try:
    #     last_output = interactive_chat(last_output)
    # except KeyboardInterrupt:
    #     break
