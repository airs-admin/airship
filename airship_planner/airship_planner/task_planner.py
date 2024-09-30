from abc import ABC, abstractmethod
import json
import logging
import requests
import yaml

#need openai library if you want to use openai model
#import openai

logging.basicConfig(level=logging.INFO, format='%(asctime)s-%(name)s-%(levelname)s: %(message)s')

class Task_Planner(ABC):

    @abstractmethod
    def llm_processor(self, inst):
        pass

    def get_tasks(self, inst):
        msg = self.llm_processor(inst)
        actions = self.llm_response_to_actions(msg)
        return actions

    def _load_yaml_map(self, filename):
        with open(filename, 'r') as file:
            yaml_dict = yaml.safe_load(file)

        yaml_map = {}
        for key, object in yaml_dict.items():
            yaml_map[object['label']] = [
                object['x'], object['y'], object['theta']
            ]

        return yaml_map

    def llm_response_to_actions(self, msg):
        actions_list = json.loads(msg)

        processed_actions = []

        for action in actions_list:
            go_to_action = action.get("go_to", "")
            if go_to_action:
                go_to_table, go_to_coords = go_to_action.split("[")
                go_to_table = go_to_table.strip()
                go_to_coords = [
                    float(coord)
                    for coord in go_to_coords.strip("]").split(", ")
                ]
                processed_actions.append(['go_to', [go_to_table, go_to_coords]])

            pick_up_action = action.get("pick_up", "")
            if pick_up_action:
                processed_actions.append(['pick_up', [pick_up_action]])

            place_action = action.get("place", "")
            if place_action:
                place_table, place_coords = place_action.split("[")
                place_table = place_table.strip()
                place_coords = [
                    float(coord)
                    for coord in place_coords.strip("]").split(", ")
                ]
                processed_actions.append(['go_to', [place_table, place_coords]])
                processed_actions.append(['place', [pick_up_action]])

        return processed_actions


class GPT4_Task_Planner(Task_Planner):

    def __init__(self, api_key, semantic_map):
        self._api_key = api_key
        self._prompt_context = """
#CONTEXT#
You are highly skilled in robotic task planning, breaking down intricate and long-term tasks into distinct primitive actions. The robot has a mobile base and an arm. The environment is represented by a semantic map in JSON format with:
{
"coordinate table": [x, y, angle]
}
where "x" and "y" are position coordinates, and "angle" represents the orientation of the table.

The following is a detailed semantic map:
"""
        self._semantic_map = self._load_yaml_map(semantic_map)

        self._prompt_skills = """

Objects (e.g., "apple") are associated with specific tables based on the semantic map. The robot needs to understand the current location, navigate to the target object, pick it up, and then bring it back to the desired location.

You must break down the language instruction into subtasks, listing a set of skills for each subtask. The output must strictly follow the format provided below, using the exact names from the semantic map and only using coordinate_table in the "go_to" and "place" subtasks.
"""
        self._prompt_objectiv = """
—SKILL—
go_to[coordinate table1] pick_up[object] place[coordinate table2] done
—SKILL—

#Modification#

    coordinate_table1 and coordinate_table2 can be the same table.

    The robot may perform more than one set of these actions, depending on the instruction.

#OUTPUT#
All of your output should be in JSON format. Only output JSON data and nothing else. Adhere strictly to the format.

The following template represents two distinct sets of actions based on the given instruction:
[
    {
        "go_to": "coordinate table1 [x1, y1, angle1]",
        "pick_up": "object1",
        "place": "coordinate table2 [x2, y2, angle2]"
    },
    {
        "go_to": "coordinate table3 [x3, y3, angle3]",
        "pick_up": "object2",
        "place": "coordinate table4 [x4, y4, angle4]"
    }
]
"""
        self._add = """
Here are my instruction:
"""

    def llm_processor(self, inst):
        prompt = self._prompt_context + json.dumps(
            self._semantic_map
        ) + self._prompt_skills + self._prompt_objectiv + self._add + inst
        # mocking GPT-4 API
        response = self.mock_gpt4_api(prompt)
        return response

    def mock_gpt4_api(self, prompt):
        '''
        openai.api_key = self._api_key
        response = openai.ChatCompletion.create(
            model="gpt-4",
            messages=[
                {"role": "user", "content": prompt}
            ]
        )
        return response['choices'][0]['message']['content']
        '''
        # mocking GPT-4 API response
        return """
        [
            {
                "go_to": "fruit table [0.61, 5.89, 90]",
                "pick_up": "apple",
                "place": "book table [-4.66, 3.4, 89]"
            },
            {
                "go_to": "book table [-4.66, 3.4, 89]",
                "pick_up": "book",
                "place": "drink table [6.97, -4.4, 78]"
            },
            {
                "go_to": "drink table [6.97, -4.4, 78]",
                "pick_up": "coke",
                "place": "fruit table [0.61, 5.89, 90]"
            }
        ]
        """


class LLAMA_Task_Planner(Task_Planner):

    def __init__(self, url, semantic_map):
        self._url = url
        self._prompt_context = """
#CONTEXT#
You are highly skilled in robotic task planning, breaking down intricate and long-term tasks into distinct primitive actions. The robot has a mobile base and an arm. The environment is represented by a semantic map in JSON format with:
{
"coordinate table": [x, y, angle]
}
where "x" and "y" are position coordinates, and "angle" represents the orientation of the table.

The following is a detailed semantic map:
"""
        self._semantic_map = self._load_yaml_map(semantic_map)

        self._prompt_skills = """

Objects (e.g., "apple") are associated with specific tables based on the semantic map. The robot needs to understand the current location, navigate to the target object, pick it up, and then bring it back to the desired location.

You must break down the language instruction into subtasks, listing a set of skills for each subtask. The output must strictly follow the format provided below, using the exact names from the semantic map and only using coordinate_table in the "go_to" and "place" subtasks.
"""
        self._prompt_objectiv = """
—SKILL—
go_to[coordinate table1] pick_up[object] place[coordinate table2] done
—SKILL—

#Modification#

    coordinate_table1 and coordinate_table2 can be the same table.

    The robot may perform more than one set of these actions, depending on the instruction.

#OUTPUT#
All of your output should be in JSON format. Only output JSON data and nothing else. Adhere strictly to the format.

The following template represents two distinct sets of actions based on the given instruction:
[
    {
        "go_to": "coordinate table1 [x1, y1, angle1]",
        "pick_up": "object1",
        "place": "coordinate table2 [x2, y2, angle2]"
    },
    {
        "go_to": "coordinate table3 [x3, y3, angle3]",
        "pick_up": "object2",
        "place": "coordinate table4 [x4, y4, angle4]"
    }
]
"""
        self._add = """
Here are my instruction:
"""

    def llm_processor(self, inst):
        prompt = self._prompt_context + json.dumps(
            self._semantic_map
        ) + self._prompt_skills + self._prompt_objectiv + self._add + inst
        response = self.ollama_api(prompt)
        return response

    def ollama_api(self, prompt):
        url = self._url
        headers = {
            "Content-Type": "application/json; charset=utf-8",
            "Accept": "application/json"
        }
        json_input = {
            "model": "llama3.1:70b",
            "messages": [{
                "role": "user",
                "content": prompt
            }],
            "stream": False
        }

        while True:
            response = requests.post(url,
                                     headers=headers,
                                     data=json.dumps(json_input))
            try:
                response_json = response.json()
                content = response_json.get("message", {}).get("content", "")
                json.loads(content)
                return content
            except (json.JSONDecodeError, ValueError):
                logging.info("Received invalid JSON content, retrying...")
                continue

