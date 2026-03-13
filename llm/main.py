import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

from tools import get_tool_declarations



checkpoint = "Qwen/Qwen2.5-0.5B-Instruct"
tokenizer = AutoTokenizer.from_pretrained(checkpoint)
model = AutoModelForCausalLM.from_pretrained(checkpoint, dtype="auto", device_map="auto")

# Chat History
messages = [
    {"role": "system", "content": "You have access to tools that move a robot."},
    {"role": "user", "content": "Move forward 2 meters"},
]

inputs = tokenizer.apply_chat_template(
    messages, 
    tools=get_tool_declarations(), 
    add_generation_prompt=True, 
    return_dict=True, 
    return_tensors="pt"
)
outputs = model.generate(**inputs.to(model.device), max_new_tokens=128)
print(tokenizer.decode(outputs[0][len(inputs["input_ids"][0]):]))