import torch
from transformers import AutoModelForCausalLM, AutoTokenizer

from tools import get_tool_declarations

SYSTEM_PROMPT = (
    "You are the director for two simulated robots: Spot and Drone. "
    "Call at most ONE function per turn. "
)



def _ask_model(model, tokenizer, messages: list):
    inputs = tokenizer.apply_chat_template(
        messages, 
        tools=get_tool_declarations(), 
        add_generation_prompt=True, 
        return_dict=True, 
        return_tensors="pt"
    )

    outputs = model.generate(
        **inputs.to(model.device), 
        max_new_tokens=128,
    )

    prompt_token_count = inputs["input_ids"].shape[1]
    generated_tokens = outputs[0][prompt_token_count:]

    return tokenizer.decode(generated_tokens, skip_special_tokens=False)


def initialize(MODEL: str):
    tokenizer = AutoTokenizer.from_pretrained(MODEL)
    model = AutoModelForCausalLM.from_pretrained(
        MODEL, 
        torch_dtype=torch.float16,
        device_map="cuda"
    )

    return tokenizer, model


# ---------- Experiment Runners ----------
def run_single_call_experiments(model, tokenizer, test_cases: list):
    results = []

    for test_case in test_cases:
        messages = [
            {"role": "system", "content": SYSTEM_PROMPT},
            {"role": "user", "content": test_case["prompt"]},
        ]

        output_text = _ask_model(
            model=model,
            tokenizer=tokenizer,
            messages=messages,
        )

        results.append(
            {
                "prompt": test_case["prompt"],
                "expected_call": test_case["expected_call"],
                "raw_output": output_text,
                "category": test_case.get("category"),
            }
        )

    return results


def run_sequential_call_experiments(model, tokenizer, prompts: list):
    all_outputs = []
    messages = [{
        "role": "system", "content": SYSTEM_PROMPT
    }]
    
    for prompt in prompts:
        messages.append({
            "role": "user", "content": prompt
        })

        output = ask_model(
            model=model, 
            tokenizer=tokenizer, 
            messages=messages
        )

        all_outputs.append(output)